# 📄 PROPOSITION DE WORKSHOPS : ROBOTIQUE AUTONOME AVEC ROS 2

**Niveau :** Master 1 (M1)

**Domaine :** Robotique Numérique / Systèmes Autonomes

**Durée :** 1 Semestre (14 semaines)

**Support Pédagogique :** Projet Open-Source "Faucon" (Robot mobile agricole sous ROS 2 Jazzy & Gazebo Sim)

---

## 1. CONTEXTE ET OBJECTIFS PÉDAGOGIQUES

Le présent Workshop a pour objectif d'immerger un étudiant (ou pas) de niveau minimum Master 1 dans la conception logicielle d'un robot autonome moderne. Au lieu d'étudier des concepts isolés, l'apprentissage s'articule autour d'un cas d'usage concret : le projet **Faucon**, un système robotique agricole simulé.

À l'issue de ce workshop, l'étudiant sera capable de :
*   Maîtriser l'architecture middleware **ROS 2** (nœuds, topics, services, paramètres).
*   Comprendre et configurer un environnement de simulation dynamique (**Gazebo Sim**).
*   Mettre en œuvre des algorithmes de traitement de données multicapteurs (GNSS, Centrale inertielle, LiDAR, Caméra RGBD).
*   Déployer une architecture complète de **Navigation Autonome** (SLAM, Nav2).
*   Développer des interfaces de supervision web temps réel.

## 2. APPROCHE PÉDAGOGIQUE

Le workshop est découpé en **7 ateliers thématiques**. Chaque atelier s'étalera sur deux semaines et sera divisé en deux sessions distinctes :

*   **Semaine A — Session de Transmission (Théorie & Démonstration) :** 
    Le formateur introduit les concepts mathématiques et algorithmiques de base. Cette session est suivie d'une démonstration pratique (Live Coding et exécution sur la stack logicielle du projet Faucon). Un Travail Pratique (TP) est assigné à l'étudiant.
*   **Semaine B — Session de Retour d'Expérience (REX & Code Review) :** 
    L'étudiant présente les résultats de son TP réalisé en autonomie. Il pourra proposer des solutions personnalisées s'il le souhaite. La séance est dédiée à l'analyse des difficultés, au débogage collaboratif, et à la validation des concepts via une revue de code (*Pull Request*).

---

## 3. PLANNING DÉTAILLÉ DES ATELIERS (SYLLABUS)

### 🛠️ Atelier 1 : Modélisation cinématique et découverte du middleware ROS 2
*   **Session 1 (Théorie) :** Graphes ROS 2, notions de repères tridimensionnels (TF2), URDF (Unified Robot Description Format) et macros Xacro. Démonstration de la visualisation géométrique sous RViz2 (`faucon_base_desc`).
*   **Session 2 (TP Autonome) :** Ajout d'un nouveau capteur (ex: LiDAR) sur le modèle URDF de la variante (les solutions personnalisées sont encouragés).

### 🎮 Atelier 2 : Création du monde 3D, simulation dynamique et contrôle
*   **Session 1 (Théorie) :** Construction d'un environnement 3D avec Gazebo Sim (fichiers SDF, modèles visuels/collisions, découverte du package `virtual_maize_field`). Moteurs physiques, intégration du robot, architecture `ros2_control` et cinématique 4WS. Lancement du modèle dynamique (`launch_base.sh`).
*   **Session 2 (TP Autonome) :** Modification du monde 3D (ex: création d'un nouveau layout de champ de maïs, ajout de dénivelés ou de nouveaux obstacles géométriques). Ensuite, développement d'un nœud Python publiant sur `/cmd_vel` pour que le robot navigue dans ce nouvel environnement personnalisé.

### 🌍 Atelier 3 : Odométrie, capteurs proprioceptifs et localisation GNSS
*   **Session 1 (Théorie) :** L'arbre des transformations (TF tree selon REP-105). Modèles d'odométrie, fusion de données locales (Odom + IMU) via filtre de Kalman Étendu (EKF local). Signaux GNSS, conversions géodésiques globales vers locales (ENU/UTM). Analyse du pipeline `gps_to_local.py`.
*   **Session 2 (TP Autonome) :** Déploiement de l'outil Mapviz (`faucon_localisation`). L'étudiant devra piloter le robot et tracer en temps réel la trajectoire de l'odométrie vs la trajectoire GNSS brute sur une carte OpenStreetMap.

### 👁️ Atelier 4 : Perception 3D et filtrage spatial
*   **Session 1 (Théorie) :** Fonctionnement des capteurs LiDAR et caméras de profondeur. Manipulation de nuages de points (PointCloud2). Algorithmes de segmentation et filtrage (PCL, RANSAC). Explication du nœud `lidar_calibrator` du projet.
*   **Session 2 (TP Autonome) :** Implémentation logicielle (Python ou C++) d'un filtre *PassThrough*. L'objectif est de tronquer les points du nuage LiDAR situés au-delà d'une certaine hauteur (feuillage du maïs) pour ne conserver que les obstacles d'intérêt.

### 🗺️ Atelier 5 : SLAM et cartographie de l'environnement
*   **Session 1 (Théorie) :** Introduction au problème du SLAM. Alignement de scans (ICP), fermeture de boucle visuelle, cartes d'occupation. **Étude architecturale TF :** gestion de l'estimation globale (`map` -> `odom`) et explication de la fusion (ou du conflit) entre les données GNSS et le graphe SLAM (RTAB-Map).
*   **Session 2 (TP Autonome) :** Lancement de l'environnement complet et téléopération du robot pour cartographier un couloir du champ virtuel. Extraction et sauvegarde de la carte 2D statique générée pour l'atelier suivant.

### 🚀 Atelier 6 : Planification de trajectoire et Navigation Autonome
*   **Session 1 (Théorie) :** Fonctionnement de la stack `Nav2`. Algorithmes de recherche de chemins globaux (A*, Dubins), contrôleurs locaux (DWA, MPPI), cartes de coûts (Costmaps) et arbres de comportements (Behavior Trees). Présentation du script `trajectory_generator.py`.
*   **Session 2 (TP Autonome) :** Configuration des waypoints GPS (`gps_waypoints.yaml`) pour définir un chemin entre les cultures. Lancement d'une mission de navigation 100% autonome d'un bout à l'autre du champ avec évitement d'obstacles dynamiques.

### 💻 Atelier 7 : Orchestration, Interfaces Utilisateurs (IHM) et Bilan
*   **Session 1 (Théorie) :** Architecture Web/ROS 2 via `rosbridge_suite`. Framework React.js, WebSockets, et streaming vidéo MJPEG. Découverte de l'interface Ground Control Station (`faucon_ihm`).
*   **Session 2 (TP Autonome) :** Ajout d'un widget personnalisé sur le Dashboard React. Création d'un bouton publiant un topic ROS 2 pour déclencher un arrêt d'urgence ou une animation spécifique sur le robot.

---

## 4. MODALITÉS D'ÉVALUATION (Proposition)

Afin de garantir une approche professionnalisante, l'étudiant sera évalué de la façon suivante :
1.  **Évaluation Continue (50%) :** Lors de chaque session REX (Semaine B), l'étudiant devra avoir poussé son code sur une branche Git dédiée. La qualité du code, l'utilisation de Git, et la capacité à expliquer les choix techniques seront notées.
2.  **Projet Final (50%) :** Lors de la 14ème semaine, l'étudiant réalisera une démonstration complète en temps réel : démarrage du système, génération d'une trajectoire via l'IHM, et supervision du robot accomplissant sa mission dans la simulation de manière robuste.

## 5. PRÉREQUIS TECHNIQUES

*   **Système :** PC sous Linux Ubuntu 24.04 (ou utilisation d'un conteneur Docker performant).
*   **Logiciels :** ROS 2 Jazzy, Gazebo Sim, Git, Visual Studio Code, Node.js (pour l'IHM).
*   **Connaissances préalables utiles :** Bases de Linux (ligne de commande), programmation orientée objet (Python et/ou C++), algèbre linéaire basique.