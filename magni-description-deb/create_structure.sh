#!/bin/bash

# Define base directory
BASE_DIR="magni-description-deb"

# Create directory structure
mkdir -p "$BASE_DIR/DEBIAN"
mkdir -p "$BASE_DIR/opt/ros/jazzy/src/magni_robot/magni_description"

# Create placeholder files
touch "$BASE_DIR/DEBIAN/control"
touch "$BASE_DIR/DEBIAN/postinst"
touch "$BASE_DIR/DEBIAN/prerm"
touch "$BASE_DIR/DEBIAN/postrm"

echo "Directory structure created successfully."
