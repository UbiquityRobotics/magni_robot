# Build instructions:
dpkg-deb --build magni-description-deb magni-description_1.0.0_all.deb

# Install the package
sudo dpkg -i magni-description_1.0.0_all.deb

# Fix any dependency issues (if they occur)
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz

sudo apt-get install -f


# --------------------------------------------------------------------------------
cd magni-description-deb
./create_deb_package.sh


Usage:

Update the build script: Change SOURCE_PATH to point to your actual source directory
Run the build script: ./build_deb_script.sh
Install the package: sudo dpkg -i magni-description_1.0.0_all.deb