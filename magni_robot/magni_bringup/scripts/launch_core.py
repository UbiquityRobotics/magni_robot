#!/usr/bin/python3

import sys, os, subprocess, time, argparse
import roslaunch, rospkg
import yaml
import smbus  # used for the hw rev stuff
import em
from collections import abc

rp = rospkg.RosPack()

# Path to the robot.yaml on the robot (not tracked by git)
conf_path = "/etc/ubiquity/robot.yaml"

# Path to the default_robot.yaml config inside magni_robot repo (git tracked)
default_conf_path = rp.get_path("magni_bringup") + "/config/default_robot.yaml"

# Path to the .em file from which the core.launch is generated
core_em_path = rp.get_path("magni_bringup") + "/launch/core_launch.em"

#Color codes for printing in shell
class clr:
    OK = '\033[92m'
    WARN = '\033[93m'
    ERROR = '\033[91m'
    ENDC = '\033[0m'


def read_yaml_safe(file_path):
    """Read YAML safely to avoid excessive CPU usage during file parsing."""
    try:
        with open(file_path) as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(clr.ERROR + f"Error reading {file_path}: {e}" + clr.ENDC)
        return None


def dict_replace_missing(d1, d2):
    """Update missing keys in d1 from d2."""
    for k in d2:
        if k not in d1:
            print(clr.WARN + f"WARN: Missing key '{k}' in config. Adding from default." + clr.ENDC)
            d1[k] = d2[k]
        elif isinstance(d2[k], dict):
            dict_replace_missing(d1[k], d2[k])


def get_config_replace_missing(conf_path, default_conf):
    """Get configuration from the YAML file, replacing missing values with defaults."""
    y_conf = read_yaml_safe(conf_path)
    if y_conf is None:
        print(clr.WARN + f"Using default configuration, as {conf_path} was not found or is empty." + clr.ENDC)
        return default_conf

    dict_replace_missing(y_conf, default_conf)
    return y_conf


def create_core_launch_file(em_path, path, conf=default_conf, camera_extrinsics_file="", lidar_extrinsics_file="", oled_display=0, board_rev=0):
    """Generate the core launch file using an EM template."""
    try:
        with open(em_path) as em_launch_file:
            em_launch = em_launch_file.read()
            expanded_em_launch = em.expand(em_launch, {
                "camera_extrinsics_file": camera_extrinsics_file,
                "lidar_extrinsics_file": lidar_extrinsics_file,
                "sonars_installed": conf["sonars_installed"],
                "shell_installed": conf["shell_installed"],
                "tower_installed": conf["tower_installed"],
                "oled_display": oled_display,
                "controller_board_version": str(board_rev),
                "serial_port": str(conf["ubiquity_motor"]["serial_port"]),
                "serial_baud": str(conf["ubiquity_motor"]["serial_baud"]),
                # (Other parameters truncated for brevity)
            })
            with open(path, 'w') as out_file:
                out_file.write(expanded_em_launch)
            return True
    except FileNotFoundError:
        print(clr.WARN + f"{em_path} does not exist!" + clr.ENDC)
        return False
    except Exception as e:
        print(clr.ERROR + f"Error generating {em_path}: {e}" + clr.ENDC)
        return False


def find_file_by_priority(first_path, second_path):
    """Return the first existing file, otherwise the second one."""
    first_path = os.path.expanduser(first_path)
    second_path = os.path.expanduser(second_path)
    if os.path.isfile(first_path):
        return first_path
    elif os.path.isfile(second_path):
        return second_path
    return ""


def feedback_popen(command, cwd):
    """Execute subprocess with feedback, ensuring minimal CPU usage."""
    try:
        proc = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=cwd)
        output, _ = proc.communicate()
        return output, proc.returncode
    except Exception as e:
        print(clr.ERROR + f"Error executing command: {e}" + clr.ENDC)
        return "", -1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true", help="Only generate ROS launch file without launching it")
    parser.add_argument("--launch_generate_path", default="/tmp/generated_core.launch", help="Generate the launch file to this path")
    arguments, _ = parser.parse_known_args()

    conf = get_config_replace_missing(conf_path, default_conf)

    # Print full config in debug mode
    if arguments.debug:
        print("DEBUG: Full content of the applied config:")
        print(conf)

    # Check if OLED display is installed
    oled_display_installed = conf["oled_display"]["controller"] == "SH1106"

    # Time synchronization
    if conf["force_time_sync"] == "True":
        time.sleep(5)  # Avoid early call during boot
        try:
            timeout = time.time() + 40
            while time.time() < timeout:
                output = subprocess.run(["pifi", "status"], check=True, text=True, capture_output=True).stdout
                if "not activated" in output:
                    time.sleep(5)
                    continue
                if "acting as an Access Point" in output:
                    print("AP mode detected. Skipping time sync.")
                    break
                if "is connected to" in output:
                    print("Network connected. Waiting for time sync.")
                    subprocess.call(["chronyc", "waitsync", "20"])
                    break
        except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
            print("Error during time sync:", e)
            subprocess.call(["chronyc", "waitsync", "6"])

    # Reading board revision (use I2C for boards >= 5.0)
    boardRev = 0
    if conf["ubiquity_motor"].get("board_version") is None:
        try:
            i2cbus = smbus.SMBus(1)
            i2cbus.write_byte(0x20, 0xFF)
            time.sleep(0.2)
            inputPortBits = i2cbus.read_byte(0x20)
            boardRev = 49 + (15 - (inputPortBits & 0x0F))
            print(f"Board revision: {boardRev}")
        except Exception as e:
            print("Error reading motor controller board version:", e)

    # Handle extrinsics files
    magni_description_path = rp.get_path("magni_description")
    camera_extr_file = lidar_extr_file = ""

    if conf.get("raspicam_position"):
        camera_extr_file = find_file_by_priority(f"~/.ros/extrinsics/camera_extrinsics_{conf['raspicam_position']}.yaml", magni_description_path + "/extrinsics/camera_extrinsics_{conf['raspicam_position']}.yaml")
    if conf.get("lidar_position"):
        lidar_extr_file = find_file_by_priority(f"~/.ros/extrinsics/lidar_extrinsics_{conf['lidar_position']}.yaml", magni_description_path + "/extrinsics/lidar_extrinsics_{conf['lidar_position']}.yaml")

    create_success = create_core_launch_file(core_em_path, arguments.launch_generate_path, conf=conf, camera_extrinsics_file=camera_extr_file, lidar_extrinsics_file=lidar_extr_file, oled_display=oled_display_installed, board_rev=boardRev)
    
    if not create_success:
        print(clr.ERROR + "ERROR: Creating launch file did not succeed" + clr.ENDC)
        return

    print(clr.OK + f"Launch file generated at {arguments.launch_generate_path}" + clr.ENDC)

    # Launch the generated launch file if not in debug mode
    if not arguments.debug:
        print(f"Launching: roslaunch {arguments.launch_generate_path}")
        sys.argv = ["roslaunch", arguments.launch_generate_path]
        roslaunch.main(sys.argv)
    else:
        print("In debug mode, skipping launch.")

        print(f"Executing 'rosrun roslaunch roslaunch-check {arguments.launch_generate_path}'")
        output, success = feedback_popen(f"rosrun roslaunch roslaunch-check {arguments.launch_generate_path}", os.environ['HOME'])
        if "FAILURE" in output:
            print(clr.ERROR + "LAUNCH CHECK FAILURE:" + clr.ENDC)
            print(output)
        else:
            print(clr.OK + "Launch check OK" + clr.ENDC)


if __name__ == "__main__":
    main()