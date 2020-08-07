#!/usr/bin/python

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

import sys, os, subprocess, time
import roslaunch
import yaml
import smbus # used for the hw rev stuff

conf_path = "/etc/ubiquity/robot.yaml"

default_conf = \
{
    'raspicam' : {'position' : 'forward'},
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

def get_conf():
    try:
        with open(conf_path) as conf_file:
            try:
                conf = yaml.safe_load(conf_file)
            except Exception as e:
                print('Error reading yaml file, using default configuration')
                print(e)
                return default_conf

            if (conf is None):
                print('WARN /etc/ubiquity/robot.yaml is empty, using default configuration')
                return default_conf

            for key, value in default_conf.items():
                if key not in conf:
                    conf[key] = value

            return conf
    except IOError:
        print("WARN /etc/ubiquity/robot.yaml doesn't exist, using default configuration")
        return default_conf
    except yaml.parser.ParserError:
        print("WARN failed to parse /etc/ubiquity/robot.yaml, using default configuration")
        return default_conf

if __name__ == "__main__":
    conf = get_conf()

    # We only support 1 version of the Sonars right now
    if conf['sonars'] == 'pi_sonar_v1':
        conf['sonars'] = 'true'
    else:
        conf['sonars'] = 'false'

    # We only support 1 display type right now
    oled_display_installed = 'false'
    if conf['oled_display']['controller'] == 'SH1106':
        oled_display_installed = 'true'

    print conf

    if conf['force_time_sync']:
        time.sleep(5) # pifi doesn't like being called early in boot
        try:
            timeout = time.time() + 40 # up to 40 seconds
            while (1):
                if (time.time() > timeout): 
                    print "Timed out"
                    raise RuntimeError # go to error handling
                output = subprocess.check_output(["pifi", "status"])
                if "not activated" in output:
                    time.sleep(5)
                if "acting as an Access Point" in output:
                    print "we are in AP mode, don't wait for time"
                    break # dont bother with chrony in AP mode
                if "is connected to" in output:
                    print "we are connected to a network, wait for time"
                    subprocess.call(["chronyc", "waitsync", "20"]) # Wait for chrony sync
                    break
        except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
            print "Error calling pifi"
            subprocess.call(["chronyc", "waitsync", "6"]) # Wait up to 60 seconds for chrony
    else:
        print "Skipping time sync steps due to configuration" 

    boardRev = 0

    if conf['motor_controller']['board_version'] == None: 
        # Code to read board version from I2C
        # The I2C chip is only present on 5.0 and newer boards
        try:
            i2cbus = smbus.SMBus(1)
            i2cbus.write_byte(0x20, 0xFF)
            time.sleep(0.2)
            inputPortBits = i2cbus.read_byte(0x20)
            boardRev = 49 + (15 - (inputPortBits & 0x0F))
            print "Got board rev: %d" % boardRev
        except: 
            print "Error reading motor controller board version from i2c"

    extrinsics_file = '~/.ros/camera_info/extrinsics_%s.yaml' % conf['raspicam']['position']
    extrinsics_file = os.path.expanduser(extrinsics_file)
    if not os.path.isfile(extrinsics_file):
        extrinsics_file = '-'

    controller = conf['motor_controller'] # make it easier to access elements
    # Ugly, but works 
    # just passing argv doesn't work with launch arguments, so we assign sys.argv
    sys.argv = ["roslaunch", "magni_bringup", "core.launch", 
                         "raspicam_mount:=%s" % conf['raspicam']['position'],
                         "sonars_installed:=%s" % conf['sonars'],
                         "controller_board_version:=%d" % boardRev,
                         "camera_extrinsics_file:=%s" % extrinsics_file,
                         "controller_serial_port:=%s" % controller['serial_port'],
                         "controller_serial_baud:=%s" % controller['serial_baud'],
                         "controller_pid_proportional:=%s" % controller['pid_proportional'],
                         "controller_pid_integral:=%s" % controller['pid_integral'],
                         "controller_pid_derivative:=%s" % controller['pid_derivative'],
                         "controller_pid_denominator:=%s" % controller['pid_denominator'],
                         "controller_pid_moving_buffer_size:=%s" % controller['pid_moving_buffer_size'],
                         "controller_pid_velocity:=%s" % controller['pid_velocity'],
                         "oled_display:=%s" % oled_display_installed
               ]

    roslaunch.main(sys.argv)

