#!/usr/bin/python3

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

import sys, os, subprocess, time, argparse
import roslaunch, rospkg
import yaml
import smbus  # used for the hw rev stuff
import em

rp = rospkg.RosPack()

# Path to the robot.yaml on the robot (not tracked by git)
conf_path_1 = "/etc/ubiquity/robot.yaml"

# Path to the default_robot.yaml config inside magni_robot repo (git tracked)
conf_path_2 = rp.get_path("magni_bringup") + "/config/default_robot.yaml"

# Path to the .em file from which the core.launch is generated
core_em_path = rp.get_path("magni_bringup") + "/launch/core_launch.em"

# We rely on the conf_path_2 always existing since its inside the repo.
# The default config is always loaded because specific elements are loaded from it if
# they are missing in conf_path_1 yaml (see get_yaml() function).
try:
    with open(conf_path_2) as cf:
        default_conf = yaml.safe_load(cf)
except Exception as e:
    print("Error reading " + conf_path_2)
    print(e)
    exit


def get_yaml(yaml_path, default_yaml):
    try:
        with open(yaml_path) as conf_file:
            try:
                y_conf = yaml.safe_load(conf_file)
            except Exception as e:
                print("Error reading yaml file, using default configuration")
                print(e)
                return default_yaml

            if y_conf is None:
                print("WARN " + yaml_path + " is empty, using default configuration")
                return default_yaml

            for key, value in default_yaml.items():
                if key not in y_conf:
                    print(
                        "WARN: Did not find '"
                        + str(key)
                        + "' in "
                        + yaml_path
                        + ". Replacing it with: "
                        + str(value)
                    )
                    y_conf[key] = value

            return y_conf
    except IOError:
        print("WARN " + yaml_path + " doesn't exist, using default configuration")
        return default_conf
    except yaml.parser.ParserError:
        print("WARN failed to parse " + yaml_path + ", using default configuration")
        return default_conf


def create_core_launch_file(
    em_path,
    path,
    conf=default_conf,
    camera_extrinsics_file="",
    lidar_extrinsics_file="",
    oled_display=0,
    board_rev=0,
):
    try:
        sonars_installed = conf["sonars"]
        motor_controller_params = conf["motor_controller"]
        # velocity controller params
        vc_params = conf["ubiquity_velocity_controller"]
    except Exception as e:
        print("There is an error with the conf: " + str(e))
        return False

    if len(motor_controller_params) != 9:
        print(
            "Motor_controller_params dictionary must contain 9 items. Instead it has:",
            str(len(motor_controller_params)),
        )
        return False

    # TODO should I be doing more checks here -> maybe if dictionary and such

    vc_lin = vc_params["linear"]["x"]
    vc_ang = vc_params["angular"]["z"]

    try:
        with open(em_path) as em_launch_file:
            em_launch = em_launch_file.read()
            expanded_em_launch = em.expand(
                em_launch,
                {
                    "camera_extrinsics_file": camera_extrinsics_file,
                    "lidar_extrinsics_file": lidar_extrinsics_file,
                    "sonars_installed": sonars_installed,
                    "oled_display": oled_display,
                    "controller_board_version": str(board_rev),
                    "serial_port": str(motor_controller_params["serial_port"]),
                    "serial_baud": str(motor_controller_params["serial_baud"]),
                    "pid_proportional": str(
                        motor_controller_params["pid_proportional"]
                    ),
                    "pid_integral": str(motor_controller_params["pid_integral"]),
                    "pid_derivative": str(motor_controller_params["pid_derivative"]),
                    "pid_denominator": str(motor_controller_params["pid_denominator"]),
                    "pid_moving_buffer_size": str(
                        motor_controller_params["pid_moving_buffer_size"]
                    ),
                    "pid_velocity": str(motor_controller_params["pid_velocity"]),
                    "wheel_separation_multiplier": str(
                        vc_params["wheel_separation_multiplier"]
                    ),
                    "wheel_radius_multiplier": str(
                        vc_params["wheel_radius_multiplier"]
                    ),
                    "has_velocity_limits": str(vc_lin["has_velocity_limits"]),
                    "has_velocity_limits": str(vc_lin["max_velocity"]),
                    "has_acceleration_limits": str(vc_lin["has_acceleration_limits"]),
                    "max_acceleration": str(vc_lin["max_acceleration"]),
                    "has_velocity_limits": str(vc_ang["has_velocity_limits"]),
                    "max_velocity": str(vc_ang["max_velocity"]),
                    "has_acceleration_limits": str(vc_ang["has_acceleration_limits"]),
                    "max_acceleration": str(vc_ang["max_acceleration"]),
                },
            )
    except FileNotFoundError:
        print("WARN " + em_path + " doesn't exist!")
        return False
    except (JSONDecodeError, NameError) as e:
        print("WARN failed to parse " + em_path)
        print(e)
        return False

    f = open(path, "w")
    f.write(expanded_em_launch)
    f.close()
    return True


# finds file path by priority:
# 1.) Checks first path and returns it if it exists
# 2.) Checks second path and returns it if it exists
# 3.) If none of them exists returns empty string
def find_file_by_priority(first_path, second_path):
    first_path = os.path.expanduser(first_path)
    try:
        if not os.path.isfile(first_path):
            # print("File "+first_path+" does not exist")
            second_path = os.path.expanduser(second_path)
            if not os.path.isfile(second_path):
                # print("File "+second_path+" does not exist, using default config")
                return ""
            else:
                # print("File "+second_path+" found")
                return second_path
        else:
            # print("File "+first_path+" found")
            return first_path
    except Exception as e:
        print("Error finding file by priority: " + e)
        return "noFileFound"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Only generate ROS launch file without launching it",
    )
    parser.add_argument(
        "--launch_generate_path",
        default="/tmp/core.launch",
        help="Generated the launch file to this path",
    )
    arguments, unknown = parser.parse_known_args()

    conf_path = find_file_by_priority(conf_path_1, conf_path_2)
    if conf_path == "":
        print(
            "ERROR: configuration file robot.yaml could not be found in either:\n"
            + conf_path_1
            + " OR\n"
            + conf_path_2
        )
        return
    else:
        print("Found config file: " + conf_path)

    conf = get_yaml(conf_path, default_conf)

    # We only support 1 version of the Sonars right now
    if conf["sonars"] == "pi_sonar_v1":
        conf["sonars"] = True
    else:
        conf["sonars"] = False

    # We only support 1 display type right now
    oled_display_installed = False
    if conf["oled_display"]["controller"] == "SH1106":
        oled_display_installed = True

    # print out the whole config if in debug mode
    if arguments.debug == True:
        print(conf)

    if conf["force_time_sync"] == "True":
        time.sleep(5)  # pifi doesn't like being called early in boot
        try:
            timeout = time.time() + 40  # up to 40 seconds
            while 1:
                if time.time() > timeout:
                    print("Timed out")
                    raise RuntimeError  # go to error handling
                output = subprocess.run(
                    ["pifi", "status"], check=True, text=True, capture_output=True
                ).stdout
                if "not activated" in output:
                    time.sleep(5)
                if "acting as an Access Point" in output:
                    print("we are in AP mode, don't wait for time")
                    break  # dont bother with chrony in AP mode
                if "is connected to" in output:
                    print("we are connected to a network, wait for time")
                    subprocess.call(
                        ["chronyc", "waitsync", "20"]
                    )  # Wait for chrony sync
                    break
        except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
            print("Error calling pifi")
            subprocess.call(
                ["chronyc", "waitsync", "6"]
            )  # Wait up to 60 seconds for chrony
    else:
        print("Skipping time sync steps due to configuration")

    boardRev = 0

    if conf["motor_controller"]["board_version"] == None:
        # Code to read board version from I2C
        # The I2C chip is only present on 5.0 and newer boards
        try:
            i2cbus = smbus.SMBus(1)
            i2cbus.write_byte(0x20, 0xFF)
            time.sleep(0.2)
            inputPortBits = i2cbus.read_byte(0x20)
            boardRev = 49 + (15 - (inputPortBits & 0x0F))
            print("Got board rev: %d" % boardRev)
        except:
            print("Error reading motor controller board version from i2c")

    """
    check for lidar and camera extrinsics files in two places with following priorities:
        1.) in ~/.ros/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml
        2.) in package magni_description/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml
    If no file was found, do not load the sensor in urdf
    """
    magni_description_path = rp.get_path("magni_description")

    camera_extr_file = ""
    lidar_extr_file = ""

    # this may happen with older robot.yaml where conf["raspicam"] did not have camera_installed entry yet
    if not "camera_installed" in conf["raspicam"]:
        print("ERROR: 'camera_installed' entry was not found in conf['raspicam']")
        return

    # check for camera extrinsics
    if (
        conf["raspicam"]["camera_installed"] == "True"
        or conf["raspicam"]["camera_installed"] == "true"
    ):
        # get camera extrinsics
        path1 = (
            "~/.ros/extrinsics/camera_extrinsics_%s.yaml" % conf["raspicam"]["position"]
        )
        path2 = (
            magni_description_path
            + "/extrinsics/camera_extrinsics_%s.yaml" % conf["raspicam"]["position"]
        )
        camera_extr_file = find_file_by_priority(path1, path2)
        if camera_extr_file == "":
            print(
                "WARN: Camera will NOT be enabled in urdf, because extrinsics file not found in neither: "
                + path1
                + " OR\n"
                + path2
            )
            # In this case, the camera_extr_file as empty string is passed to create the core.launch. Upon ros-launching that, URDF
            # detects that the extrinsics yaml path string is empty and does not load it into robot_description. That is why it is important
            # that this string is "" if any error with getting extrinsics.
        else:
            print("Camera extrinsics found: " + camera_extr_file)

    # check for lidar extrinsics
    if (
        conf["lidar"]["lidar_installed"] == "True"
        or conf["lidar"]["lidar_installed"] == "true"
    ):
        # get lidar extrinsics
        path1 = "~/.ros/extrinsics/lidar_extrinsics_%s.yaml" % conf["lidar"]["position"]
        path2 = (
            magni_description_path
            + "/extrinsics/lidar_extrinsics_%s.yaml" % conf["lidar"]["position"]
        )
        lidar_extr_file = find_file_by_priority(path1, path2)
        if lidar_extr_file == "":
            print(
                "WARN: Lidar will NOT be enabled in urdf, because extrinsics file not found in neither: "
                + path1
                + " OR\n"
                + path2
            )
            # In this case, the lidar_extr_file as empty string is passed to create the core.launch. Upon ros-launching that, URDF
            # detects that the extrinsics yaml path string is empty and does not load it into robot_description. That is why it is important
            # that this string is "" if any error with getting extrinsics.
        else:
            print("Lidar extrinsics found: " + lidar_extr_file)

    create_success = create_core_launch_file(
        core_em_path,
        arguments.launch_generate_path,
        conf=conf,
        camera_extrinsics_file=camera_extr_file,
        lidar_extrinsics_file=lidar_extr_file,
        oled_display=oled_display_installed,
        board_rev=boardRev,
    )

    if not create_success:
        print("ERROR: Creating launch file did not succeed")
        return
    else:
        print("Launch file generated at " + arguments.launch_generate_path)

    # only launch the generated launch if not in debug mode
    if arguments.debug != True:
        print("Launching with command: roslaunch " + arguments.launch_generate_path)
        sys.argv = ["roslaunch", arguments.launch_generate_path]
        roslaunch.main(sys.argv)
    else:
        print("In debug mode the generated roslaunch is not launched")


if __name__ == "__main__":
    main()
