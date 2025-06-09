# Build instructions:
dpkg-deb --build magni-description-deb magni-description_1.0.0_all.deb

# Install the package
sudo dpkg -i magni-description_1.0.0_all.deb

# Fix any dependency issues (if they occur)
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros

sudo apt-get install -f