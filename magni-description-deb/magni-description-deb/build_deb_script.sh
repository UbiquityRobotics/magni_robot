#!/bin/bash
# Script to create the Debian package with proper structure
set -e

PACKAGE_NAME="magni-description"
VERSION="1.0.0"
ROS_DISTRO="jazzy"
DEB_DIR="./${PACKAGE_NAME}-deb"
# Auto-detect source path or set manually
if [ -d "$HOME/ros2_ws/src/magni_robot/magni_description" ]; then
    SOURCE_PATH="$HOME/ros2_ws/src/magni_robot/magni_description"
elif [ -d "./magni_description" ]; then
    SOURCE_PATH="./magni_description"
elif [ -d "../magni_description" ]; then
    SOURCE_PATH="../magni_description"
else
    echo "Error: Cannot find magni_description source directory!"
    echo "Please check these locations:"
    echo "  $HOME/ros2_ws/src/magni_robot/magni_description"
    echo "  ./magni_description"
    echo "  ../magni_description"
    echo ""
    echo "Or manually set SOURCE_PATH in this script to the correct location."
    exit 1
fi

echo "Building Debian package for ${PACKAGE_NAME}..."

# Clean up previous build
rm -rf ${DEB_DIR}
rm -f ${PACKAGE_NAME}_${VERSION}_all.deb

# Create directory structure
mkdir -p ${DEB_DIR}/DEBIAN
mkdir -p ${DEB_DIR}/opt/ros/${ROS_DISTRO}/src/magni_robot

# Copy source files to the package structure
echo "Copying source files from: ${SOURCE_PATH}"
echo "Checking source directory contents:"
ls -la "${SOURCE_PATH}" || {
    echo "Error: Source directory does not exist or is not accessible: ${SOURCE_PATH}"
    exit 1
}

# Verify essential files exist
if [ ! -f "${SOURCE_PATH}/package.xml" ]; then
    echo "Error: package.xml not found in ${SOURCE_PATH}"
    exit 1
fi

if [ ! -f "${SOURCE_PATH}/CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found in ${SOURCE_PATH}"
    exit 1
fi

cp -r "${SOURCE_PATH}" ${DEB_DIR}/opt/ros/${ROS_DISTRO}/src/magni_robot/magni_description

# Create control file
cat > ${DEB_DIR}/DEBIAN/control << EOF
Package: ${PACKAGE_NAME}
Version: ${VERSION}
Architecture: all
Maintainer: Your Name <your.email@example.com>
Depends: ros-${ROS_DISTRO}-rclcpp, python3-colcon-common-extensions, python3-rosdep
Description: Magni robot description package
 URDF and mesh files for the Magni robot.
 This package contains source files that will be compiled during installation.
EOF

# Create preinst script
cat > ${DEB_DIR}/DEBIAN/preinst << 'EOF'
#!/bin/bash
set -e

PACKAGE_NAME="magni_description"
ROS_DISTRO="jazzy"
SOURCE_DIR="/opt/ros/${ROS_DISTRO}/src/magni_robot/${PACKAGE_NAME}"
BACKUP_DIR="/tmp/${PACKAGE_NAME}_backup_$$"

case "$1" in
    install|upgrade)
        # If this is an upgrade and source files exist, back them up
        if [ -d "${SOURCE_DIR}" ]; then
            echo "Backing up existing source files..."
            mkdir -p "$(dirname ${BACKUP_DIR})"
            cp -r "${SOURCE_DIR}" "${BACKUP_DIR}"
            echo "Backup created at: ${BACKUP_DIR}"
        fi
        ;;
    abort-upgrade)
        ;;
    *)
        echo "preinst called with unknown argument \`$1'" >&2
        exit 1
        ;;
esac

exit 0
EOF

# Create postinst script (copy the enhanced version from the previous artifact)
cat > ${DEB_DIR}/DEBIAN/postinst << 'EOF'
#!/bin/bash
set -e

PACKAGE_NAME="magni_description"
ROS_DISTRO="jazzy"
TEMP_WS="/tmp/magni_build_ws"
SOURCE_DIR="/opt/ros/${ROS_DISTRO}/src/magni_robot/${PACKAGE_NAME}"
INSTALL_DIR="/opt/ros/${ROS_DISTRO}/share/${PACKAGE_NAME}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to cleanup on exit
cleanup() {
    if [ -d "${TEMP_WS}" ]; then
        log_info "Cleaning up temporary workspace..."
        rm -rf ${TEMP_WS}
    fi
    # Clean up any backup files older than 1 hour
    find /tmp -name "${PACKAGE_NAME}_backup_*" -type d -mmin +60 -exec rm -rf {} + 2>/dev/null || true
}

trap cleanup EXIT

case "$1" in
    configure)
        log_info "Configuring ${PACKAGE_NAME} for ROS2 ${ROS_DISTRO}..."

        # Check if running as root
        if [ "$EUID" -ne 0 ]; then
            log_error "This script must be run as root"
            exit 1
        fi

        # Check if ROS2 is installed
        if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
            log_error "ROS2 ${ROS_DISTRO} not found. Please install ROS2 ${ROS_DISTRO} first."
            exit 1
        fi

        # Verify source directory exists
        if [ ! -d "${SOURCE_DIR}" ]; then
            log_error "Source directory not found: ${SOURCE_DIR}"
            exit 1
        fi

        # Verify essential files
        if [ ! -f "${SOURCE_DIR}/package.xml" ] || [ ! -f "${SOURCE_DIR}/CMakeLists.txt" ]; then
            log_error "Essential package files missing in ${SOURCE_DIR}"
            exit 1
        fi

        # Source ROS2 environment
        source /opt/ros/${ROS_DISTRO}/setup.bash

        # Check and install build tools if needed
        MISSING_TOOLS=()
        if ! command -v colcon &> /dev/null; then
            MISSING_TOOLS+=("python3-colcon-common-extensions")
        fi
        if ! command -v rosdep &> /dev/null; then
            MISSING_TOOLS+=("python3-rosdep")
        fi

        if [ ${#MISSING_TOOLS[@]} -gt 0 ]; then
            log_info "Installing missing build tools: ${MISSING_TOOLS[*]}"
            apt-get update
            apt-get install -y "${MISSING_TOOLS[@]}"
        fi

        # Initialize rosdep if needed
        if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
            rosdep init || true
        fi

        # Build process
        log_info "Building ${PACKAGE_NAME}..."
        rm -rf ${TEMP_WS}
        mkdir -p ${TEMP_WS}/src/magni_robot
        cp -r ${SOURCE_DIR} ${TEMP_WS}/src/magni_robot/
        cd ${TEMP_WS}

        rosdep update || true
        rosdep install --from-paths src --ignore-src -r -y || true

        if ! colcon build --packages-select ${PACKAGE_NAME} --cmake-args -DCMAKE_BUILD_TYPE=Release; then
            log_error "Build failed"
            exit 1
        fi

        # Install
        log_info "Installing to system..."
        mkdir -p /opt/ros/${ROS_DISTRO}/share /opt/ros/${ROS_DISTRO}/lib
        [ -d "${INSTALL_DIR}" ] && rm -rf ${INSTALL_DIR}
        cp -r ${TEMP_WS}/install/${PACKAGE_NAME}/* /opt/ros/${ROS_DISTRO}/

        # Update package index
        mkdir -p /opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/packages
        touch /opt/ros/${ROS_DISTRO}/share/ament_index/resource_index/packages/${PACKAGE_NAME}

        # Set permissions
        chown -R root:root ${INSTALL_DIR} ${SOURCE_DIR}
        find ${INSTALL_DIR} -type d -exec chmod 755 {} \;
        find ${INSTALL_DIR} -type f -exec chmod 644 {} \;
        find ${SOURCE_DIR} -type d -exec chmod 755 {} \;
        find ${SOURCE_DIR} -type f -exec chmod 644 {} \;

        log_info "Installation completed successfully!"
        log_info "Source: ${SOURCE_DIR}"
        log_info "Installed: ${INSTALL_DIR}"
        ;;

    abort-upgrade|abort-remove|abort-deconfigure)
        ;;
    *)
        echo "postinst called with unknown argument \`$1'" >&2
        exit 1
        ;;
esac

exit 0
EOF

# Make scripts executable
chmod 755 ${DEB_DIR}/DEBIAN/preinst
chmod 755 ${DEB_DIR}/DEBIAN/postinst

# Build the package
echo "Building Debian package..."
dpkg-deb --build ${DEB_DIR} ${PACKAGE_NAME}_${VERSION}_all.deb

echo "Debian package created: ${PACKAGE_NAME}_${VERSION}_all.deb"
echo ""
echo "To install:"
echo "  sudo dpkg -i ${PACKAGE_NAME}_${VERSION}_all.deb"
echo ""
echo "To remove:"
echo "  sudo dpkg -r ${PACKAGE_NAME}"