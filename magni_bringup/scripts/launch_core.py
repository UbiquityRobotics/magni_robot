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

"""
We rely on the default_conf_path always existing since its inside the repo.
The default config is always loaded because specific elements are loaded from it if
they are missing in conf_path yaml (see get_config_replace_missing() function).
"""
try:
    with open(default_conf_path) as cf:
        default_conf = yaml.safe_load(cf)
except Exception as e:
    print(clr.ERROR + "Error reading " + default_conf_path + clr.ENDC)
    print(e)
    exit

"""
If a key in `d2` is not in `d1`, add it to `d1` with the value from `d2`.
Any keys that are in `d1` but not in `d2` are left alone. `d2` is unchanged.
`d1` and `d2` can be nested dictionaries.

:param d1: The dictionary that you want to add/replace missing keys to
:param d2: The default configuration dictionary
"""
def dict_replace_missing(d1, d2):
    for k in d2:
        if k in d1:
            if type(d2[k]) is dict:
                dict_replace_missing(d1[k],d2[k])
        else:
            print(
                clr.WARN
                + "WARN: Did not find '"
                + str(k)
                + "' in "
                + conf_path
                + ". Adding it with value: "
                + str(d2[k])
                + clr.ENDC
            )
            d1[k] = d2[k]


"""
It reads a yaml file and if any key is missing in the yaml file, it replaces it with the default
value

:param conf_path: The path to the configuration file
:param default_conf: This is the default configuration, where replacement keys are taken from
:return: A dictionary with the configuration
"""
def get_config_replace_missing(conf_path, default_conf):
    try:
        with open(conf_path) as conf_file:
            try:
                y_conf = yaml.safe_load(conf_file)
            except Exception as e:
                print(clr.ERROR + 
                    + "Error reading yaml file from "
                    + conf_path
                    + ", using default configuration"
                    + clr.ENDC)
                print(e)
                return default_conf

            if y_conf is None:
                print("WARN " + conf_path + " is empty, using default configuration")
                return default_conf

            print(clr.OK + "Found config file: " + conf_path + clr.ENDC)

            # if any key missing in y_conf replace it individually from default_conf
            dict_replace_missing(y_conf, default_conf)

            # print(y_conf)    
            return y_conf
    except IOError:
        print(clr.WARN 
            + "WARN "
            + conf_path 
            + " doesn't exist, using default configuration"
            + clr.ENDC)
        return default_conf
    except yaml.parser.ParserError:
        print("WARN failed to parse " + conf_path + ", using default configuration")
        return default_conf


"""
It takes a template file, and replaces all the variables in it with the values in the `conf`
dictionary

:param em_path: The path to the em file to use as a template
:param path: The path to the launch file to be created
:param conf: The configuration dictionary
:param camera_extrinsics_file: The path to the camera extrinsics file
:param lidar_extrinsics_file: The path to the lidar extrinsics file
:param oled_display: oled_display number, defaults to 0 (optional)
:param board_rev: The revision of the controller board, defaults to 0 (optional)
:return: A boolean value of success - if true the generation of launch file at path was successfull.
"""
def create_core_launch_file(
    em_path,
    path,
    conf=default_conf,
    camera_extrinsics_file="",
    lidar_extrinsics_file="",
    oled_display=0,
    board_rev=0,
):
    mot_cont = conf["ubiquity_motor"]
    vel_cont = conf["ubiquity_velocity_controller"]
    joint_pub = conf["ubiquity_joint_publisher"]
    
    try:
        with open(em_path) as em_launch_file:
            em_launch = em_launch_file.read()
            expanded_em_launch = em.expand(
                em_launch,
                {
                    "camera_extrinsics_file": camera_extrinsics_file,
                    "lidar_extrinsics_file": lidar_extrinsics_file,
                    "sonars_installed": conf["sonars_installed"],
                    "shell_installed": conf["shell_installed"],
                    "tower_installed": conf["tower_installed"],
                    "oled_display": oled_display,
                    "controller_board_version": str(board_rev),

                    "serial_port": str(mot_cont["serial_port"]),
                    "serial_baud": str(mot_cont["serial_baud"]),
                    "serial_loop_rate": str(mot_cont["serial_loop_rate"]),
                    "controller_loop_rate": str(mot_cont["controller_loop_rate"]),
                    "pid_proportional": str(mot_cont["pid_proportional"]),
                    "pid_integral": str(mot_cont["pid_integral"]),
                    "pid_derivative": str(mot_cont["pid_derivative"]),
                    "pid_denominator": str(mot_cont["pid_denominator"]),
                    "pid_control": str(mot_cont["pid_control"]),
                    "drive_type": str(mot_cont["drive_type"]),
                    "wheel_type": str(mot_cont["wheel_type"]),
                    "wheel_gear_ratio": str(mot_cont["wheel_gear_ratio"]),
                    "fw_max_pwm": str(mot_cont["fw_max_pwm"]),
                    "fw_max_speed_fwd": str(mot_cont["fw_max_speed_fwd"]),
                    "fw_max_speed_rev": str(mot_cont["fw_max_speed_rev"]),
                    "pid_moving_buffer_size": str(mot_cont["pid_moving_buffer_size"]),
                    "pid_velocity": str(mot_cont["pid_velocity"]),
                    
                    "joint_controller_type": str(joint_pub["type"]),
                    "joint_controller_publish_rate": str(joint_pub["publish_rate"]),

                    "vel_controller_type": str(vel_cont["type"]),
                    "left_wheel": str(vel_cont["left_wheel"]),
                    "right_wheel": str(vel_cont["right_wheel"]),
                    "publish_rate": str(vel_cont["publish_rate"]),
                    "pose_covariance_diagonal": str(vel_cont["pose_covariance_diagonal"]),
                    "twist_covariance_diagonal": str(vel_cont["twist_covariance_diagonal"]),
                    "cmd_vel_timeout": str(vel_cont["cmd_vel_timeout"]),
                    "enable_odom_tf": str(vel_cont["enable_odom_tf"]),
                    "wheel_separation": str(vel_cont["wheel_separation"]),
                    "base_frame_id": str(vel_cont["base_frame_id"]),

                    "wheel_separation_multiplier": str(vel_cont["wheel_separation_multiplier"]),
                    "wheel_radius": str(vel_cont["wheel_radius"]),
                    "wheel_radius_multiplier": str(vel_cont["wheel_radius_multiplier"]),
                    "lin_has_velocity_limits": str(vel_cont["linear"]["x"]["has_velocity_limits"]),
                    "lin_max_velocity": str(vel_cont["linear"]["x"]["max_velocity"]),
                    "lin_has_acceleration_limits": str(vel_cont["linear"]["x"]["has_acceleration_limits"]),
                    "lin_max_acceleration": str(vel_cont["linear"]["x"]["max_acceleration"]),

                    "ang_has_velocity_limits": str(vel_cont["angular"]["z"]["has_velocity_limits"]),
                    "ang_max_velocity": str(vel_cont["angular"]["z"]["max_velocity"]),
                    "ang_has_acceleration_limits": str(vel_cont["angular"]["z"]["has_acceleration_limits"]),
                    "ang_max_acceleration": str(vel_cont["angular"]["z"]["max_acceleration"]),
                },
            )
    except FileNotFoundError:
        print(clr.WARN + "WARN " + em_path + " doesn't exist!" + clr.ENDC)
        return False
    except Exception as e:
        print(clr.ERROR + "ERROR failed to parse " + em_path + clr.ENDC)
        print(e)
        return False

    f = open(path, "w")
    f.write(expanded_em_launch)
    f.close()
    return True


"""
If the first file exists, return it, otherwise return the second file. 
If none of them exists returns empty string.

:param first_path: The path to the file you want to find
:param second_path: The path to the default config file
:return: The path to the file that was found.
"""
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

"""
Execute popen with feedback
"""
def feedback_popen(command, cwd):
    proc = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=True, cwd=cwd)
    return proc.communicate()[0], proc.returncode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Only generate ROS launch file without launching it",
    )
    parser.add_argument(
        "--launch_generate_path",
        default="/tmp/generated_core.launch",
        help="Generate the launch file to this path",
    )
    arguments, unknown = parser.parse_known_args()

    conf = get_config_replace_missing(conf_path, default_conf)

    # print out the whole config if in debug mode
    if arguments.debug == True:
        print("DEUBG: Full content of the applied config:")
        print(conf)

    # We only support 1 display type right now
    oled_display_installed = False
    if conf["oled_display"]["controller"] == "SH1106":
        oled_display_installed = True

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

    if conf["ubiquity_motor"]["board_version"] == None:
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

    # check for camera extrinsics
    if (conf["raspicam_position"] != None and conf["raspicam_position"] != "None"):
        # get camera extrinsics
        path1 = (
            "~/.ros/extrinsics/camera_extrinsics_%s.yaml" % conf["raspicam_position"]
        )
        path2 = (
            magni_description_path
            + "/extrinsics/camera_extrinsics_%s.yaml" % conf["raspicam_position"]
        )
        camera_extr_file = find_file_by_priority(path1, path2)
        if camera_extr_file == "":
            print(
                clr.WARN
                +"WARN: Camera will NOT be enabled in urdf, because extrinsics file not found in neither: "
                + path1
                + " OR\n"
                + path2
                + clr.ENDC
            )
            # In this case, the camera_extr_file as empty string is passed to create the core.launch. Upon ros-launching that, URDF
            # detects that the extrinsics yaml path string is empty and does not load it into robot_description. That is why it is important
            # that this string is "" if any error with getting extrinsics.
        else:
            print(clr.OK + "Camera enabled with extrinsics: " + camera_extr_file + clr.ENDC)
    else:
        print("Camera not enabled in robot.yaml")

    # check for lidar extrinsics
    if (conf["lidar_position"] != None and conf["lidar_position"] != "None"):
        # get lidar extrinsics
        path1 = "~/.ros/extrinsics/lidar_extrinsics_%s.yaml" % conf["lidar_position"]
        path2 = (
            magni_description_path
            + "/extrinsics/lidar_extrinsics_%s.yaml" % conf["lidar_position"]
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
            print(clr.OK + "Lidar enabled with extrinsics:: " + lidar_extr_file + clr.ENDC)
    else:
        print("Lidar not enabled in robot.yaml")

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
        print(clr.ERROR + "ERROR: Creating launch file did not succeed" + clr.ENDC)
        return
    else:
        print(clr.OK + "Launch file generated at " + arguments.launch_generate_path + clr.ENDC)

    # only launch the generated launch if not in debug mode
    if arguments.debug != True:
        print("Launching with command: roslaunch " + arguments.launch_generate_path)
        sys.argv = ["roslaunch", arguments.launch_generate_path]
        roslaunch.main(sys.argv)
    else:
        print("In debug mode the generated roslaunch is not launched")

        print("Executing 'rosrun roslaunch roslaunch-check " + arguments.launch_generate_path+"'")
        output, success = feedback_popen("rosrun roslaunch roslaunch-check " + arguments.launch_generate_path, os.environ['HOME'])
        if str(output).find("FAILURE") > 0:
            print(clr.ERROR
                + "LAUNCH CHECK FAILURE:")
            print(output)
        else:
            print(clr.OK
                + "Launch check OK"
                + clr.ENDC)
        

if __name__ == "__main__":
    main()
