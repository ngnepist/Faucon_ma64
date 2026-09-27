#!/usr/bin/env python3

"""
ROS 2 node — Mission Lifecycle Manager
Ce nœud est le gestionnaire de haut niveau d'une mission autonome. Son rôle principal est de :
1. Recevoir un chemin sous forme de waypoints GPS.
2. Vérifier que les waypoints sont valides.
3. Vérifier que le datum GNSS est disponible.
4. Convertir les coordonnées GPS en coordonnées locales ENU.
5. Construire un nav_msgs/Path.
6. Envoyer ce chemin à Nav2 via l'action FollowPath.
7. Gérer le cycle de vie de la mission :
      IDLE → LOADING → READY → RUNNING → COMPLETED
                              ↓
                            PAUSED
8. Permettre START / STOP / PAUSE / RESUME.
9. Surveiller la présence de l'odométrie.
10. Publier l'état de la mission vers l'IHM.
"""

from __future__ import annotations
# ============================================================================
# IMPORTS PYTHON
# ============================================================================
import json          # Permet de construire le message JSON publié sur /mission/status
import math          # Fonctions mathématiques, notamment sin() et cos()
import os            # Manipulation des chemins de fichiers
import sys           # Modification du PYTHONPATH
import time          # Utilisé pour le watchdog de l'odométrie
import uuid          # Génération d'un identifiant unique de mission

from typing import List, Optional
# ============================================================================
# IMPORTS ROS 2
# ============================================================================
import rclpy
from rclpy.node import Node
# ActionClient permet de communiquer avec une action ROS 2.
# Ici, on utilise l'action FollowPath de Nav2.
from rclpy.action import ActionClient
# QoS utilisés notamment pour le datum GNSS et le chemin publié.
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)
# États standard d'une action ROS 2 :
# SUCCEEDED, ABORTED, CANCELED, etc.
from action_msgs.msg import GoalStatus

# ============================================================================
# TYPES DE MESSAGES ROS
# ============================================================================
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from geographic_msgs.msg import GeoPoint

# ============================================================================
# SERVICES / ACTIONS UTILISÉS
# ============================================================================
# Service de robot_localization permettant de convertir
# plusieurs coordonnées Latitude/Longitude/Altitude en coordonnées cartésiennes locales.
from robot_localization.srv import FromLLArray, ToLL
from std_srvs.srv import Trigger
#  custom FollowPath action is used to avoid dependency on nav2_msgs package
from faucon_interfaces.action import NavigateFauconMission

# ============================================================================
# IMPORT DU MODULE MISSION_CORE
# ============================================================================
# mission_core.py se trouve dans le même dossier que ce script.
#
# On récupère ici :
#   - State            → les différents états de la machine à états
#   - validate_waypoints → validation du chemin GPS
#   - nearest_wp_index → recherche du waypoint le plus proche du robot
#
# __file__ correspond au chemin du script actuellement exécuté.
_DIR = os.path.dirname(os.path.abspath(__file__))

# On ajoute le dossier du script au PYTHONPATH si nécessaire.
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)
from mission_core import (State, validate_waypoints, nearest_wp_index)

# ============================================================================
# QoS "LATCHED"
# ============================================================================
# Ce QoS permet à un nouveau subscriber de récupérer la dernière valeur publiée.
# C'est particulièrement utile pour :
#   - /gnss/datum
#   - /mission/path
# TRANSIENT_LOCAL :
# le publisher conserve la dernière valeur.
#
# RELIABLE :
# ROS 2 essaie de garantir la livraison du message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

# ============================================================================
# CLASSE PRINCIPALE
# ============================================================================
class MissionManager(Node):
    """
    Gestionnaire du cycle de vie complet d'une mission autonome.
    Topics consommés :
        /mission/load_path
            std_msgs/String
            Contenu YAML du chemin GPS
        /mission/command
            std_msgs/String
            START | STOP | PAUSE | RESUME
        /gnss/datum
            sensor_msgs/NavSatFix
            Indique que le datum GNSS est disponible
        odometry/global
            nav_msgs/Odometry
            Pose courante du robot
    Topics produits :
        /mission/status
            std_msgs/String
            État de la mission au format JSON, 5 Hz
        /mission/path
            nav_msgs/Path
            Chemin ENU complet
    """

    # ========================================================================
    # INITIALISATION
    # ========================================================================
    def __init__(self):
        # Initialisation du Node ROS 2.
        # Le nom du nœud sera :
        #     /mission_manager
        super().__init__("mission_manager")
        # ====================================================================
        # MACHINE À ÉTATS
        # ====================================================================
        # État initial du gestionnaire.
        # Le robot démarre en attente d'une mission.
        self._state = State.IDLE
        # Message d'erreur courant.
        self._error_msg = ""
        # Identifiant unique de la mission.
        self._mission_id = ""

        # ====================================================================
        # DONNÉES DE LA MISSION / DU CHEMIN
        # ====================================================================
        self._raw_wps: List[dict] = []
        # Waypoints originaux après conversion GPS → ENU.
        self._mission_points: List[Point] = []
        # Chemin complet après conversion GPS → ENU et densification.
        # Il s'agit d'un nav_msgs/Path.
        self._full_path: Optional[Path] = None
        self._gps_path_points = []
        self._gps_path_index = 0

        # Nombre total de waypoints.
        self._total_wp = 0
        # Index du waypoint actuellement considéré comme atteint.
        self._current_wp = 0
        self._mission_total_distance = 0.0
        self._distance_remaining = 0.0
        self._mission_progress = 0.0 # avancement de la mission en pourcentage (0.0 → 1.0)
        # Première pose du chemin de mission.
        self._mission_start_point: Optional[PoseStamped] = None

        # ====================================================================
        # ÉTAT DU ROBOT
        # ====================================================================
        # Dernière position connue du robot.
        self._robot_pose: Optional[PoseStamped] = None
        # True lorsque le datum GNSS a été reçu.
        self._datum_ready = False
        # Temps de réception de la dernière odométrie.
        # Utilisé par le watchdog.
        self._last_odom_t = 0.0

        #===================================================================
        # PARAMETRES ROS 2
        #===================================================================
        self.declare_parameter("mission_start_distance_threshold", 0.5)

        # ====================================================================
        # CLIENT ACTION NAV2
        # ====================================================================
        # Création du client d'action NavigateFauconMission.
        self._goal_handle = None
        self._pause_requested = False
        self._stop_requested = False
        self._nav_client = ActionClient(self, NavigateFauconMission, "/navigate_faucon_mission")

        # ====================================================================
        # SERVICES
        # ====================================================================
        # Client du service robot_localization :
        #     /fromLLArray (resp. /toLL)
        # Ce service convertit des coordonnées :
        #     Latitude / Longitude / Altitude (resp. x,y z)
        # en :
        #     x / y / z (resp. Latitude, Longitude, Altitude)
        # dans le repère local.
        self._fromll = self.create_client(FromLLArray, "/fromLLArray")
        self._to_ll_client = self.create_client(ToLL, "/toLL")
        # Timer utilisé lorsqu'on doit attendre que le service /fromLLArray soit disponible.
        self._fromll_retry_timer = None
        self._srv_near_mission_start = self.create_service(Trigger,"/is_robot_near_mission_start",
            self._on_is_robot_near_mission_start,
        )

        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================
        self.create_subscription(String, "/mission/load_path", self._on_load_path, 10)
        self.create_subscription(String,"/mission/command",self._on_command,10)
        self.create_subscription(NavSatFix, "/gnss/datum", self._on_datum, LATCHED_QOS)
        self.create_subscription(Odometry, "odometry/global", self._on_odom, 10)

        # ===================================================================
        # PUBLISHERS
        # ====================================================================
        self._pub_status = self.create_publisher(String, "/mission/status", 10)
        self._pub_path = self.create_publisher(Path, "/mission/path", LATCHED_QOS)
        self._pub_path_gps = self.create_publisher(String, "/mission/path_gps", LATCHED_QOS,)

        # ====================================================================
        # TIMERS
        # ====================================================================
        # Publication du statut toutes les 0.2 secondes.
        # 1 / 0.2 = 5 Hz
        self.create_timer( 0.2, self._publish_status)
        # Exécution du watchdog toutes les 2 secondes.
        self.create_timer( 2.0, self._watchdog)
        # Message indiquant que le node est prêt.
        self.get_logger().info( "mission_manager prêt - état: IDLE")

    # ========================================================================
    # CALLBACK DATUM GNSS
    # ========================================================================
    def _on_datum(self, _: NavSatFix) -> None:
        """
        Callback exécuté lorsqu'un datum GNSS est reçu.
        Le datum indique que la référence géographique nécessaire
        aux conversions GPS → coordonnées locales est disponible.
        """
        # On ne fait cette opération qu'une seule fois.
        if not self._datum_ready:
            self._datum_ready = True
            self.get_logger().info("Datum GNSS reçu - conversions GPS→ENU disponibles.")

    # ========================================================================
    # CALLBACK ODOMÉTRIE
    # ========================================================================
    def _on_odom(self, msg: Odometry) -> None:
        """
        Callback exécuté à chaque réception d'une odométrie.
        Il permet :
            - de mémoriser la pose du robot ;
            - de mettre à jour le waypoint courant ;
            - de nourrir le watchdog.
        """
        # Conversion Odometry → PoseStamped.
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        # Sauvegarde de la pose actuelle.
        self._robot_pose = ps
        # Sauvegarde du temps de dernière réception.
        # time.monotonic() est utilisé plutôt que time.time() car il est adapté aux mesures de durée.
        self._last_odom_t = time.monotonic()
        # ====================================================================
        # MISE À JOUR DU WAYPOINT COURANT
        # ====================================================================
        # On ne cherche le waypoint courant que pendant l'exécution.
        if self._state == State.RUNNING and self._mission_points:
            positions = [(p.x, p.y) for p in self._mission_points]
            idx = nearest_wp_index(positions, ps.pose.position.x,ps.pose.position.y,)
            if idx > self._current_wp:
                self._current_wp = idx
            
    # ========================================================================
    # CALLBACK CHARGEMENT DU CHEMIN
    # ========================================================================
    def _on_load_path(self, msg: String) -> None:
        """
        Reçoit une nouvelle mission GPS. Une nouvelle mission n'est pas acceptée pendant RUNNING.
        """
        # Impossible de remplacer directement une mission en cours.
        if self._state == State.RUNNING:
            self.get_logger().warn("Mission en cours - envoyez STOP avant de charger un nouveau chemin.")
            return
        # Si la mission était en pause, on annule d'abord le goal Nav2.
        if self._state == State.PAUSED:
            self._cancel_goal()
        # Commence le chargement et la conversion.
        self._begin_loading(msg.data)

    def _path_length(self, path: Path) -> float:
        distance = 0.0

        for i in range(len(path.poses) - 1):
            p0 = path.poses[i].pose.position
            p1 = path.poses[i + 1].pose.position
            distance += math.hypot(p1.x - p0.x, p1.y - p0.y)

        return distance

    # ========================================================================
    # CALLBACK COMMANDES
    # ========================================================================
    def _on_command(self, msg: String) -> None:
        """
        Reçoit les commandes de l'IHM :
            START
            STOP
            PAUSE
            RESUME
        Le dictionnaire permet de sélectionner directement
        la fonction correspondant à la commande.
        """
        # Suppression des espaces et conversion en majuscules.
        cmd = msg.data.strip().upper()
        # Table de correspondance :
        # START  → _cmd_start()
        # STOP   → _cmd_stop()
        # PAUSE  → _cmd_pause()
        # RESUME → _cmd_resume()
        # Si la commande est inconnue, on affiche un warning.
        {
            "START": self._cmd_start,
            "STOP": self._cmd_stop,
            "PAUSE": self._cmd_pause,
            "RESUME": self._cmd_resume,
        }.get(cmd, lambda: self.get_logger().warn(f"Commande inconnue: '{cmd}'"),)()

    # =========================================================================
    # COMMANDES DE MISSION
    # =========================================================================
    def _cmd_start(self) -> None:
        """
        Démarre la mission. START n'est accepté que lorsque la mission est READY.
        """
        # Vérification de l'état.
        if self._state != State.READY:
            self.get_logger().warn(f"START ignoré — état: {self._state}")   
            return
        # On recommence le suivi à partir du premier waypoint.
        self._current_wp = 0
        # Envoi du chemin à Nav2.
        self._send_path(self._full_path)

    # -------------------------------------------------------------------------
    # STOP
    # -------------------------------------------------------------------------
    def _cmd_stop(self) -> None:
        if self._state not in (
            State.RUNNING,
            State.PAUSED,
            State.READY,
            State.ERROR,
        ):
            self.get_logger().warn(f"STOP ignoré — état: {self._state}")
            return

        # Aucun goal Nav2 actif : nettoyage immédiat.
        if self._goal_handle is None:
            self._finish_stop()
            return

        self._stop_requested = True

        self.get_logger().info(
            f"[{self._mission_id}] Demande d'arrêt de la mission..."
        )

        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_stop_cancelled)
    
    def _on_stop_cancelled(self, future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self._stop_requested = False
            return self._set_error(
                f"Erreur pendant l'annulation Nav2 pour STOP: {e}"
            )

        if not response.goals_canceling:
            self._stop_requested = False
            return self._set_error(
                "Nav2 a refusé l'annulation pour STOP"
            )

        self._goal_handle = None
        self._stop_requested = False

        self.get_logger().info(
            f"[{self._mission_id}] Goal Nav2 annulé pour STOP."
        )

        self._finish_stop()

    def _finish_stop(self) -> None:
        self._transition(State.ABORTED)
        self._reset()
        self._transition(State.IDLE)

        self.get_logger().info(
            "Mission arrêtée et réinitialisée."
        )

    # -------------------------------------------------------------------------
    # PAUSE
    # -------------------------------------------------------------------------
    def _cmd_pause(self) -> None:
        if self._state != State.RUNNING:
            self.get_logger().warn(f"PAUSE ignoré — état: {self._state}")
            return

        if self._goal_handle is None:
            self.get_logger().warn("PAUSE impossible — aucun goal Nav2 actif.")
            return

        self._pause_requested = True

        self.get_logger().info(
            f"[{self._mission_id}] Demande de pause..."
        )
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_pause_cancelled)

    def _on_pause_cancelled(self, future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self._pause_requested = False
            return self._set_error(
                f"Erreur pendant l'annulation Nav2: {e}"
            )

        if not response.goals_canceling:
            self._pause_requested = False
            return self._set_error(
                "Nav2 a refusé l'annulation pour PAUSE"
            )
        self._goal_handle = None
        self._pause_requested = False
        self._transition(State.PAUSED)
        self.get_logger().info(
            f"[{self._mission_id}] Mission en pause."
        )

    # -------------------------------------------------------------------------
    # RESUME
    # -------------------------------------------------------------------------
    def _cmd_resume(self) -> None:
        if self._state != State.PAUSED:
            self.get_logger().warn(f"RESUME ignoré — état: {self._state}")
            return

        if self._full_path is None:
            return self._set_error("RESUME impossible — aucune mission chargée")

        self.get_logger().info(
            f"[{self._mission_id}] Reprise de la mission."
        )

        self._send_path(self._full_path)

    # =========================================================================
    # CHARGEMENT ET CONVERSION GPS → ENU
    # =========================================================================
    def _begin_loading(self, yaml_str: str) -> None:
        """
        Commence le chargement d'une mission.
        Étapes :
            1. Passage à LOADING.
            2. Validation des waypoints.
            3. Vérification du datum GNSS.
            4. Sauvegarde de la mission.
            5. Conversion GPS → ENU.
        """
        # Passage dans l'état LOADING.
        self._transition(State.LOADING)
        # ---------------------------------------------------------------------
        # Validation du YAML / des waypoints
        # ---------------------------------------------------------------------
        try:
            wps = validate_waypoints(yaml_str)
        except ValueError as e:
            # Toute erreur de validation provoque l'état ERROR.
            return self._set_error(str(e))
        # ---------------------------------------------------------------------
        # Vérification du datum GNSS
        # ---------------------------------------------------------------------
        if not self._datum_ready:
            return self._set_error("Datum GNSS non disponible — localisation non initialisée" )
        # ---------------------------------------------------------------------
        # Sauvegarde des données de mission
        # ---------------------------------------------------------------------
        self._raw_wps = wps
        self._total_wp = len(wps)
        self._current_wp = 0
        # Génération d'un ID unique de mission.
        # uuid4().hex produit une longue chaîne hexadécimale.
        # [:8] permet de ne garder que les 8 premiers caractères.
        self._mission_id = uuid.uuid4().hex[:8].upper()
        self.get_logger().info(f"[{self._mission_id}] {self._total_wp} waypoints validés. Conversion GPS→ENU...")
        # Démarrage de la conversion GPS → ENU.
        self._start_conversion()

    # =========================================================================
    # ATTENTE DU SERVICE FROMLLARRAY
    # =========================================================================
    def _start_conversion(self) -> None:
        """
        Vérifie si le service /fromLLArray est disponible.
        """
        if self._fromll.service_is_ready():
            # Le service est déjà disponible.
            self._do_fromll()
        else:
            # Sinon, on attend et on réessaie périodiquement.
            self.get_logger().info( "Attente du service /fromLLArray...")
            self._fromll_retry_timer = self.create_timer(1.0,self._retry_fromll,)

    # -------------------------------------------------------------------------
    # Nouvelle tentative de connexion au service
    # -------------------------------------------------------------------------
    def _retry_fromll(self) -> None:
        """
        Vérifie périodiquement si /fromLLArray est maintenant disponible.
        """
        # Si on n'est plus en LOADING, on n'a plus besoin du timer.
        if self._state != State.LOADING:
            self._cancel_timer(
                self._fromll_retry_timer
            )
            return
        # Si le service est maintenant disponible...
        if self._fromll.service_is_ready():
            # ...on arrête le timer.
            self._cancel_timer(
                self._fromll_retry_timer
            )
            # Et on effectue la conversion.
            self._do_fromll()

    # =========================================================================
    # CONVERSION GPS → ENU
    # =========================================================================
    def _do_fromll(self) -> None:
        """
        Prépare et envoie la requête au service /fromLLArray.
        """
        # Création d'une requête.
        req = FromLLArray.Request()
        # Conversion des dictionnaires Python contenant :
        #
        # latitude
        # longitude
        # altitude
        #
        # en messages geographic_msgs/GeoPoint.
        req.ll_points = [
            GeoPoint(
                latitude=float(w["latitude"]),
                longitude=float(w["longitude"]),
                altitude=float(
                    w.get("altitude", 0.0)
                ),
            )
            for w in self._raw_wps
        ]
        # Appel asynchrone du service.
        # Le programme ne bloque pas ici en attendant la réponse.
        # Lorsque la réponse arrive, ROS 2 appelle :
        #     _on_fromll_response()
        self._fromll.call_async(req).add_done_callback(self._on_fromll_response)

    # =========================================================================
    # VÉRIFICATION DE LA POSITION DU ROBOT PAR RAPPORT AU DÉBUT DE MISSION
    # =========================================================================
    def _on_is_robot_near_mission_start(self, request: Trigger.Request, response: Trigger.Response,) -> Trigger.Response:
        """
        Service ROS 2 permettant de savoir si le robot est proche du début de mission.
        """
        # Une mission doit être chargée.
        if self._mission_start_point is None:
            response.success = False
            response.message = "Aucun début de mission disponible."
            return response
        # La position actuelle du robot doit être connue.
        if self._robot_pose is None:
            response.success = False
            response.message = "Pose actuelle du robot indisponible."
            return response
        # Position actuelle du robot.
        rx = self._robot_pose.pose.position.x
        ry = self._robot_pose.pose.position.y
        # Position du début de mission.
        sx = self._mission_start_point.pose.position.x
        sy = self._mission_start_point.pose.position.y
        # Distance robot <-> début de mission.
        distance = math.hypot(rx - sx,ry - sy,)
        # Seuil configuré.
        threshold = self.get_parameter("mission_start_distance_threshold").value
        # Résultat de la condition.
        response.success = distance <= threshold
        response.message = (
            f"distance={distance:.3f} m, "
            f"threshold={threshold:.3f} m"
        )
        return response

    # =========================================================================
    # RÉPONSE DU SERVICE GPS → ENU
    # =========================================================================
    def _on_fromll_response(self, future) -> None:
        """
        Traite la réponse du service /fromLLArray.
        """

        try:
            # Récupération de la réponse.
            resp = future.result()
        except Exception as e:
            # Une erreur de service met la mission en ERROR.
            return self._set_error(f"Erreur /fromLLArray: {e}")

        # Les points convertis sont maintenant des coordonnées locales.
        pts: List[Point] = resp.map_points
        # Vérification importante : on doit recevoir exactement autant de points que de waypoints envoyés.
        if len(pts) != self._total_wp:
            return self._set_error(f"/fromLLArray: {len(pts)} points reçus pour {self._total_wp} attendus")

        # sauvegarde des points de mission convertis.
        self._mission_points = list(pts)
        # Construction du nav_msgs/Path.
        self._full_path = self._make_path(pts)


        self._mission_total_distance = self._path_length(self._full_path)
        self._distance_remaining = self._mission_total_distance
        self._mission_start_point = self._full_path.poses[0]

        # Publication du chemin complet.
        self._pub_path.publish(self._full_path)
        # publication de la version gps à l'IHM
        self._publish_path_gps()

        # La mission est maintenant prête.
        self._transition(State.READY)

        # Affichage du premier point pour information.
        p0 = self._full_path.poses[0].pose.position
        self.get_logger().info(f"[{self._mission_id}] Chemin ENU prêt — {self._total_wp} WP. Origine: x={p0.x:.2f}, y={p0.y:.2f}. Envoyez START.")

    # =========================================================================
    # CONSTRUCTION DU NAV_MSGS/PATH
    # =========================================================================
    def _make_path(self, pts: List[Point]) -> Path:
        """
        Construit un nav_msgs/Path dense à partir des waypoints ENU.
        Les waypoints de mission restent inchangés, mais des poses intermédiaires sont ajoutées 
        pour fournir à Nav2/RPP un chemin suffisamment dense.
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        now = path.header.stamp
        # Espacement maximal entre deux poses du Path Nav2.
        interpolation_step = 0.25  # mètres
        for i in range(len(pts) - 1):
            p0 = pts[i]
            p1 = pts[i + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            distance = math.hypot(dx, dy)
            # Nombre de morceaux nécessaires pour respecter approximativement interpolation_step.
            n = max(1, math.ceil(distance / interpolation_step))
            # Orientation tangentielle au segment.
            yaw = math.atan2(dy, dx)
            q = Quaternion()
            q.z = math.sin(yaw * 0.5)
            q.w = math.cos(yaw * 0.5)
            for j in range(n):
                t = j / n
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.header.stamp = now
                ps.pose.position.x = p0.x + t * dx
                ps.pose.position.y = p0.y + t * dy
                ps.pose.position.z = 0.0
                ps.pose.orientation = q
                path.poses.append(ps)

        # ---------------------------------------------------------
        # Ajout explicite du dernier waypoint
        # ---------------------------------------------------------
        last = pts[-1]
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = now
        ps.pose.position.x = last.x
        ps.pose.position.y = last.y
        ps.pose.position.z = 0.0
        # Pour le dernier point, on conserve le yaw demandé dans le fichier de mission.
        yaw = float(self._raw_wps[-1].get("yaw", 0.0))
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        ps.pose.orientation = q
        path.poses.append(ps)
        self.get_logger().info(f"Path Nav2 densifié : {len(pts)} waypoints → {len(path.poses)} poses")
        return path

    # =========================================================================
    # PUBLICATION DU PATH DENSIFIÉ À L'IHM POUR AFFICHAGE
    # =========================================================================
    def _publish_path_gps(self) -> None:
        if self._full_path is None or not self._full_path.poses:
            self.get_logger().warn("Impossible de convertir le path GPS : path vide.")
            return
        if not self._to_ll_client.service_is_ready():
            self.get_logger().warn("Service /toLL indisponible.")
            return

        self._gps_path_points = []
        self._gps_path_index = 0
        self.get_logger().info(f"Conversion du path densifié vers GPS : {len(self._full_path.poses)} poses.")
        self._convert_next_path_pose()

    def _convert_next_path_pose(self) -> None:
        if self._full_path is None:
            return
        if self._gps_path_index >= len(self._full_path.poses):
            self._finish_path_gps_conversion()
            return

        pose = self._full_path.poses[self._gps_path_index].pose.position
        req = ToLL.Request()
        req.map_point.x = pose.x
        req.map_point.y = pose.y
        req.map_point.z = pose.z
        future = self._to_ll_client.call_async(req)
        future.add_done_callback(self._on_to_ll_response)

    def _on_to_ll_response(self, future) -> None:
        try:
            response = future.result()
            self._gps_path_points.append({"lat": response.ll_point.latitude, "lon": response.ll_point.longitude,"alt": response.ll_point.altitude,})
            self._gps_path_index += 1
            self._convert_next_path_pose()
        except Exception as e:
            self.get_logger().error(f"Erreur pendant la conversion /toLL : {e}")

    def _finish_path_gps_conversion(self) -> None:
        msg = String()
        msg.data = json.dumps({"mission_id": self._mission_id, "points": self._gps_path_points,})
        self._pub_path_gps.publish(msg)
        self.get_logger().info(f"Path GPS publié : {len(self._gps_path_points)} points.")

    # =========================================================================
    # EXÉCUTION NAV2
    # =========================================================================
    def _send_path(self, path: Optional[Path]) -> None:
        """Envoie un chemin de mission au FauconMissionNavigator."""
        if not path or not path.poses:
            return self._set_error("Chemin vide — impossible d'exécuter")
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return self._set_error("Serveur d'action /navigate_faucon_mission indisponible (timeout 5 s)")
        goal = NavigateFauconMission.Goal()
        goal.mission_path = path
        self._transition(State.RUNNING)
        self.get_logger().info(f"[{self._mission_id}] NavigateFauconMission envoyé — {len(path.poses)} poses.")
        self._nav_client.send_goal_async(goal, feedback_callback=self._on_feedback,).add_done_callback(
            self._on_goal_accepted
        )

    # =========================================================================
    # FEEDBACK NAV2
    # =========================================================================
    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._distance_remaining = feedback.distance_remaining
        if feedback.restarting:
            self._current_wp = 0
        if self._mission_total_distance <= 0.0:
            self._mission_progress = 0.0
            return
        progress = 1.0 - (self._distance_remaining / self._mission_total_distance)
        self._mission_progress = max(0.0, min(1.0, progress))

    # =========================================================================
    # ACCEPTATION DU GOAL NAV2
    # =========================================================================
    def _on_goal_accepted(self, future) -> None:
        """
        Callback appelé lorsque Nav2 répond à la demande NavigateFauconMission.
        """
        # Récupération du handle du goal.
        self._goal_handle = future.result()
        # Nav2 peut refuser le goal.
        if not self._goal_handle.accepted:
            return self._set_error( "Goal NavigateFauconMission refusé par Nav2")
        # Goal accepté.
        self.get_logger().info(f"[{self._mission_id}] Goal accepté par Nav2.")
        # On demande maintenant le résultat final.
        # Le callback _on_result() sera appelé lorsque Nav2
        # aura terminé ou abandonné le NavigateFauconMission.
        self._goal_handle.get_result_async().add_done_callback(self._on_result)

    # =========================================================================
    # RÉSULTAT FINAL NAV2
    # =========================================================================
    def _on_result(self, future) -> None:
        """
        Traite le résultat final de l'action NavigateFauconMission.
        """
        try:
            status = future.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        if self._pause_requested and status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"[{self._mission_id}] Goal Nav2 annulé pour PAUSE.")
            return
        if self._stop_requested and status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"[{self._mission_id}] Goal Nav2 annulé pour STOP.")
            return
        if self._state != State.RUNNING:
            return
        self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._transition(State.COMPLETED)
            self.get_logger().info(f"[{self._mission_id}] Mission terminée avec succès!")
            self.create_timer(3.0, self._auto_idle)
        else:
            self._set_error(f"Nav2 a terminé avec statut: {status}")

    # =========================================================================
    # RETOUR AUTOMATIQUE À IDLE
    # =========================================================================
    def _auto_idle(self) -> None:
        if self._state == State.COMPLETED:
            # Nettoyage des données.
            self._reset()
            # Retour à l'état initial.
            self._transition(State.IDLE)

    # =========================================================================
    # ANNULATION DU GOAL NAV2
    # =========================================================================
    def _cancel_goal(self) -> None:
        """
        Annule le NavigateFauconMission actuellement actif.
        """
        if self._goal_handle is not None:
            # Demande d'annulation asynchrone.
            self._goal_handle.cancel_goal_async()
            # Suppression de la référence locale.
            self._goal_handle = None

    # =========================================================================
    # WATCHDOG ODOMÉTRIE
    # =========================================================================
    def _watchdog(self) -> None:
        """
        Surveille la présence de l'odométrie.
        Le timer appelle cette fonction toutes les 2 secondes.
        Si aucune odométrie n'a été reçue depuis plus de 5 secondes, un WARNING est affiché.
        """
        # Le watchdog est pertinent uniquement pendant RUNNING.
        if (self._state == State.RUNNING and self._last_odom_t > 0.0):
            # Temps écoulé depuis la dernière odométrie.
            age = (time.monotonic() - self._last_odom_t)
            # Si plus de 5 secondes se sont écoulées...
            if age > 5.0:
                self.get_logger().warn(f"[{self._mission_id}] Odométrie absente depuis {age:.1f} s — vérifiez la localisation.")

    # =========================================================================
    # PUBLICATION DU STATUS
    # =========================================================================
    def _publish_status(self) -> None:
        """
        Publie l'état courant de la mission à 5 Hz.
        """
        progress = self._mission_progress
        # Création du message String.
        msg = String()
        if progress == 0 :
            self._current_wp = 0
        # Construction du JSON.
        msg.data = json.dumps({
            # État courant de la FSM.
            "state": self._state.value,
            # Identifiant de la mission.
            "mission_id": self._mission_id,
            # Progression entre 0 et 1.
            "progress": round(progress, 3),
            # Waypoint courant.
            "current_wp": self._current_wp,
            # Nombre total de waypoints.
            "total_wp": self._total_wp,
            # Message d'erreur éventuel.
            "error": self._error_msg,
        })
        # Publication.
        self._pub_status.publish(msg)

    # =========================================================================
    # TRANSITION D'ÉTAT
    # =========================================================================
    def _transition(self, new: State) -> None:
        # Lorsqu'on passe vers un état normal, on efface l'ancien message d'erreur.
        if new != State.ERROR:
            self._error_msg = ""
        # Log de la transition.
        self.get_logger().info(f"[mission] {self._state.value} → {new.value}")
        # Mise à jour réelle de l'état.
        self._state = new

    # =========================================================================
    # GESTION DES ERREURS
    # =========================================================================
    def _set_error(self, msg: str) -> None:
        """
        Place la FSM dans l'état ERROR. Lorsqu'une erreur critique survient :
            1. log de l'erreur ;
            2. sauvegarde du message ;
            3. annulation du goal Nav2 ;
            4. passage à ERROR.
        """
        self.get_logger().error( f"[mission] {msg}")
        # Sauvegarde du message pour l'IHM.
        self._error_msg = msg
        # Annulation de Nav2.
        self._cancel_goal()
        # Passage en ERROR.
        self._state = State.ERROR

    # =========================================================================
    # RESET
    # =========================================================================
    def _reset(self) -> None:
        """
        Réinitialise toutes les données liées à la mission.
        """
        # Suppression du chemin.
        self._full_path = None
        self._mission_points = []
        self._mission_start_point = None
        # Suppression des waypoints GPS.
        self._raw_wps = []
        # Remise à zéro du nombre de waypoints.
        self._total_wp = 0
        # Remise à zéro de l'index courant.
        self._current_wp = 0
        self._mission_total_distance = 0.0
        self._distance_remaining = 0.0
        self._mission_progress = 0.0
        # Suppression du handle Nav2.
        self._goal_handle = None
        # Suppression de l'identifiant de mission.
        self._mission_id = ""
        # Suppression du message d'erreur.
        self._error_msg = ""

    # =========================================================================
    # ANNULATION D'UN TIMER
    # =========================================================================
    @staticmethod
    def _cancel_timer(timer) -> None:
        """
        Annule proprement un timer ROS 2 s'il existe.
        """
        if timer is not None:
            timer.cancel()

# ============================================================================
# MAIN
# ============================================================================
def main():
    """
    Point d'entrée du programme.
    """
    # Initialisation du middleware ROS 2.
    rclpy.init()
    # Création du MissionManager.
    node = MissionManager()
    # Boucle principale ROS 2. C'est ici que ROS 2 traite :
    #   - les messages reçus ;
    #   - les callbacks ;
    #   - les timers ;
    #   - les réponses des services ;
    #   - les réponses des actions.
    rclpy.spin(node)
    # Lorsque le programme s'arrête :
    # destruction du node.
    node.destroy_node()
    # Arrêt propre de ROS 2.
    rclpy.shutdown()

# ============================================================================
# LANCEMENT DU PROGRAMME
# ============================================================================
if __name__ == "__main__":
    main()