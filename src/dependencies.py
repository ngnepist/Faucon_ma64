#!/usr/bin/env python3
"""
Script to create a ROS2 'dependencies' package that contains all dependencies
from packages present in the same directory as the script (recursively).
"""

import os
import sys
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import catkin_pkg.packages


def indent_xml(elem, level=0, indent_str='  '):
    """
    Indent XML element tree (fallback for Python < 3.9).

    Args:
        elem: XML element
        level: Current indentation level
        indent_str: String to use for indentation
    """
    i = '\n' + level * indent_str
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent_str
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for child in elem:
            indent_xml(child, level + 1, indent_str)
        if not child.tail or not child.tail.strip():
            child.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def collect_all_dependencies(packages):
    """
    Collects all unique dependencies from all packages.

    Args:
        packages: Dictionary of packages (returned by find_packages)

    Returns:
        List of unique dependency names
    """
    all_dependencies = set()

    for path, pkg in packages.items():
        # Collect build_depends
        for dep in pkg.build_depends:
            all_dependencies.add(str(dep))

        # Collect buildtool_depends
        for dep in pkg.buildtool_depends:
            all_dependencies.add(str(dep))

        # Collect exec_depends
        for dep in pkg.exec_depends:
            all_dependencies.add(str(dep))

        # Collect test_depends
        for dep in pkg.test_depends:
            all_dependencies.add(str(dep))

        # Collect doc_depends
        for dep in pkg.doc_depends:
            all_dependencies.add(str(dep))

    # Sort and return the list
    return sorted(list(all_dependencies))


def create_package_xml(package_dir, package_name, dependencies, maintainer_name, maintainer_email, license_name):
    """
    Creates a package.xml file for the dependencies package.

    Args:
        package_dir: Directory where to create the package.xml
        package_name: Name of the package
        dependencies: List of dependency names
        maintainer_name: Name of the maintainer
        maintainer_email: Email of the maintainer
        license_name: License name
    """
    package_xml_path = os.path.join(package_dir, 'package.xml')

    # Create root element (same format as unicore_gnss_driver/package.xml)
    root = ET.Element('package', format='3')

    # Add name
    name_elem = ET.SubElement(root, 'name')
    name_elem.text = package_name

    # Add version
    version_elem = ET.SubElement(root, 'version')
    version_elem.text = '0.0.0'

    # Add description
    description_elem = ET.SubElement(root, 'description')
    description_elem.text = 'Package containing all dependencies from workspace packages'

    # Add maintainer
    maintainer_elem = ET.SubElement(root, 'maintainer', email=maintainer_email)
    maintainer_elem.text = maintainer_name

    # Add license
    license_elem = ET.SubElement(root, 'license')
    license_elem.text = license_name

    # Add buildtool_depend
    buildtool_depend = ET.SubElement(root, 'buildtool_depend')
    buildtool_depend.text = 'ament_cmake'

    # Add dependencies
    for dep in sorted(dependencies):
        depend_elem = ET.SubElement(root, 'depend')
        depend_elem.text = dep

    # Add export
    export_elem = ET.SubElement(root, 'export')
    build_type_elem = ET.SubElement(export_elem, 'build_type')
    build_type_elem.text = 'ament_cmake'

    # Write XML file with proper formatting
    tree = ET.ElementTree(root)
    # Use ET.indent if available (Python 3.9+), otherwise use fallback
    if hasattr(ET, 'indent'):
        ET.indent(tree, space='  ')
    else:
        indent_xml(root)
    tree.write(package_xml_path, encoding='utf-8', xml_declaration=True)

    # Add XML schema declaration manually (ET doesn't preserve it correctly)
    with open(package_xml_path, 'r') as f:
        content = f.read()

    # Replace the first line with proper XML declaration and schema
    content = content.replace(
        '<?xml version=\'1.0\' encoding=\'utf-8\'?>',
        '<?xml version="1.0"?>\n<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>'
    )

    with open(package_xml_path, 'w') as f:
        f.write(content)


def create_cmakelists_txt(package_dir, package_name, dependencies):
    """
    Creates a CMakeLists.txt file for the dependencies package.

    Args:
        package_dir: Directory where to create the CMakeLists.txt
        package_name: Name of the package
        dependencies: List of dependency names
    """
    cmakelists_path = os.path.join(package_dir, 'CMakeLists.txt')

    # Generate CMakeLists.txt content
    content = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
"""

    content += """
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
"""

    with open(cmakelists_path, 'w') as f:
        f.write(content)


def create_dependencies_package(script_dir):
    """
    Creates a ROS2 'dependencies' package with all collected dependencies.

    Args:
        script_dir: Directory where the script is located (and where to search for packages)
    """
    package_name = 'dependencies'
    package_path = os.path.join(script_dir, package_name)

    # Remove the dependencies package if it exists (before searching for packages)
    # to avoid collecting dependencies from the generated package itself
    if os.path.exists(package_path):
        print(f"The directory {package_path} already exists. Removing...")
        shutil.rmtree(package_path)

    print(f"Searching for packages in: {script_dir}")

    # Find all packages recursively
    packages = catkin_pkg.packages.find_packages(script_dir)

    if not packages:
        print(f"No packages found in {script_dir}")
        return

    print(f"Found {len(packages)} package(s)")

    # Collect all dependencies
    all_dependencies = collect_all_dependencies(packages)
    print(f"Found {len(all_dependencies)} unique dependency(ies)")

    # Create package directory
    os.makedirs(package_path, exist_ok=True)
    print(f"Creating package '{package_name}' in {script_dir}")

    # Create package.xml
    create_package_xml(
        package_path,
        package_name,
        all_dependencies,
        maintainer_name='NGNEPIEPAYE WEMBE Stephane',
        maintainer_email='sngnepiepayewembe@gmail.com',
        license_name='FAUCON'
    )
    print(f"  - package.xml created")

    # Create CMakeLists.txt
    create_cmakelists_txt(package_path, package_name, all_dependencies)
    print(f"  - CMakeLists.txt created")

    print(f"Package '{package_name}' created successfully in {package_path}")
    print(f"  - {len(all_dependencies)} dependency(ies) added")


def main():
    """Main entry point of the script."""
    # Get the directory where the script is located
    script_path = Path(__file__).resolve()
    script_dir = str(script_path.parent)

    create_dependencies_package(script_dir)


if __name__ == "__main__":
    main()