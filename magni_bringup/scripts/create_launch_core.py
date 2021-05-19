#!/usr/bin/python

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

import sys, os, subprocess, time
import roslaunch
import yaml
import smbus # used for the hw rev stuff

conf_path = "/home/janez/Documents/Projects/ubiquityrobotics/magni_ws/src/magni_robot/magni_description/param/robot.yaml"#"/etc/ubiquity/robot.yaml"

default_conf = \
{
    'raspicam' : {'position' : 'upward'},
    'sonars' : 'None',
    'motor_controller' : {
        'board_version' : None,   
        'serial_port': "/dev/ttyAMA0",
        'serial_baud': 38400,
        'pid_proportional': 5000,
        'pid_integral': 7,
        'pid_derivative': -110,
        'pid_denominator': 1000,
        'pid_moving_buffer_size': 70,
        'pid_velocity': 1500
    },
    'force_time_sync' : 'True',
    'oled_display': {
        'controller': None
    }
}


default_camera_extrinsics = \
{
    'x' : '0.0',
    'y' : '0.0',
    'z' : '0.0',
    'roll' : '0.0',
    'pitch' : '0.0',
    'yaw' : '0.0'
}

def get_yaml(path, default_yaml):
    try:
        with open(path) as conf_file:
            try:
                conf = yaml.safe_load(conf_file)
            except Exception as e:
                print('Error reading yaml file, using default configuration')
                print(e)
                return default_yaml

            if (conf is None):
                print('WARN ' + path + ' is empty, using default configuration')
                return default_yaml

            for key, value in default_yaml.items():
                if key not in conf:
                    conf[key] = value

            return conf
    except IOError:
        print("WARN " + path + " doesn't exist, using default configuration")
        return default_conf
    except yaml.parser.ParserError:
        print("WARN failed to parse " + path + ", using default configuration")
        return default_conf

def create_core_launch_file(path, camera_extrinsics, camera_installed=True, lidar_installed = False, sonars_installed=False):
    print (path)

    if len(camera_extrinsics)!=6:
        print("Camera extrinsics dictionaty must contain 6 items. Instead it has:", str(len(camera_extrinsics)))
        return False

    # create if not exsisting, overwrite stuff thats already there
    f = open(path, "w")
    f.write("<launch>\n")

    # add the robot description part
    f.write("""
    <include file="$(find magni_description)/launch/description.launch">
        <arg name="camera_installed" value=" """ +str(camera_installed)+ """ "/>
        <arg name="lidar_installed" value=" """ +str(lidar_installed)+ """ "/>
        <arg name="sonars_installed" value=" """ +str(sonars_installed)+ """ "/>
        <arg name="camera_extr_x" value=" """ + str(camera_extrinsics['x']) + """ "/>
        <arg name="camera_extr_y" value=" """ + str(camera_extrinsics['y']) + """ "/>
        <arg name="camera_extr_z" value=" """ + str(camera_extrinsics['z']) + """ "/>
        <arg name="camera_extr_roll" value=" """ + str(camera_extrinsics['roll']) + """ "/>
        <arg name="camera_extr_pitch" value=" """ + str(camera_extrinsics['pitch']) + """ "/>
        <arg name="camera_extr_yaw" value=" """ + str(camera_extrinsics['yaw']) + """ "/>
    </include>
    """)


    f.write("</launch>")
    f.close()

    return True
    


if __name__ == "__main__":
    print (conf_path)
    conf = get_yaml(conf_path, default_conf)

    # # We only support 1 version of the Sonars right now
    # if conf['sonars'] == 'pi_sonar_v1':
    #     conf['sonars'] = 'true'
    # else:
    #     conf['sonars'] = 'false'

    # # We only support 1 display type right now
    # oled_display_installed = 'false'
    # if conf['oled_display']['controller'] == 'SH1106':
    #     oled_display_installed = 'true'

    print conf

    # if conf['force_time_sync']:
    #     time.sleep(5) # pifi doesn't like being called early in boot
    #     try:
    #         timeout = time.time() + 40 # up to 40 seconds
    #         while (1):
    #             if (time.time() > timeout): 
    #                 print "Timed out"
    #                 raise RuntimeError # go to error handling
    #             output = subprocess.check_output(["pifi", "status"])
    #             if "not activated" in output:
    #                 time.sleep(5)
    #             if "acting as an Access Point" in output:
    #                 print "we are in AP mode, don't wait for time"
    #                 break # dont bother with chrony in AP mode
    #             if "is connected to" in output:
    #                 print "we are connected to a network, wait for time"
    #                 subprocess.call(["chronyc", "waitsync", "20"]) # Wait for chrony sync
    #                 break
    #     except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
    #         print "Error calling pifi"
    #         subprocess.call(["chronyc", "waitsync", "6"]) # Wait up to 60 seconds for chrony
    # else:
    #     print "Skipping time sync steps due to configuration" 

    # boardRev = 0

    # if conf['motor_controller']['board_version'] == None: 
    #     # Code to read board version from I2C
    #     # The I2C chip is only present on 5.0 and newer boards
    #     try:
    #         i2cbus = smbus.SMBus(1)
    #         i2cbus.write_byte(0x20, 0xFF)
    #         time.sleep(0.2)
    #         inputPortBits = i2cbus.read_byte(0x20)
    #         boardRev = 49 + (15 - (inputPortBits & 0x0F))
    #         print "Got board rev: %d" % boardRev
    #     except: 
    #         print "Error reading motor controller board version from i2c"

    # camera_extrinsics_file = '~/.ros/camera_info/extrinsics_%s.yaml' % conf['raspicam']['position']
    camera_extrinsics_file = '/home/janez/Documents/Projects/ubiquityrobotics/magni_ws/src/magni_robot/magni_description/param/camera_extrinsics_%s.yaml' % conf['raspicam']['position']
    camera_extrinsics_file = os.path.expanduser(camera_extrinsics_file)
    if not os.path.isfile(camera_extrinsics_file):
        print("File"+camera_extrinsics_file+"does not exsist, using default confing")
        camera_extrinsics_file = 0

    camera_extrinsics = get_yaml(camera_extrinsics_file, default_camera_extrinsics)

    print (camera_extrinsics)

    # controller = conf['motor_controller'] # make it easier to access elements
    
    # # Ugly, but works 
    # # just passing argv doesn't work with launch arguments, so we assign sys.argv
    # sys.argv = ["roslaunch", "magni_bringup", "core.launch", 
    #                      "raspicam_mount:=%s" % conf['raspicam']['position'],
    #                      "sonars_installed:=%s" % conf['sonars'],
    #                      "controller_board_version:=%d" % boardRev,
    #                      "camera_extrinsics_file:=%s" % camera_extrinsics_file,
    #                      "controller_serial_port:=%s" % controller['serial_port'],
    #                      "controller_serial_baud:=%s" % controller['serial_baud'],
    #                      "controller_pid_proportional:=%s" % controller['pid_proportional'],
    #                      "controller_pid_integral:=%s" % controller['pid_integral'],
    #                      "controller_pid_derivative:=%s" % controller['pid_derivative'],
    #                      "controller_pid_denominator:=%s" % controller['pid_denominator'],
    #                      "controller_pid_moving_buffer_size:=%s" % controller['pid_moving_buffer_size'],
    #                      "controller_pid_velocity:=%s" % controller['pid_velocity'],
    #                      "oled_display:=%s" % oled_display_installed
    #            ]

    # roslaunch.main(sys.argv)
   
    create_success = create_core_launch_file(os.environ['HOME']+"/neki.launch",
                                            camera_extrinsics,
                                            camera_installed=conf['raspicam']['camera_installed'])

    if not create_success:
        print("creating launch file did not sucsseed")
    
