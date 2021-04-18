#!/usr/bin/python

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

import sys, os, subprocess, time
import roslaunch
import yaml
import smbus # used for the hw rev stuff
import copy

conf_path = "/etc/ubiquity/robot.yaml"

default_conf = \
{
    'raspicam' : {'position' : 'forward'},
    'sonars' : None,
    'motor_controller' : {
        'serial_port': "/dev/ttyAMA0",
        # 'serial_baud': 38400,
        'pid_proportional': 5000,
        'pid_integral': 2,
        'pid_derivative': -100,
        #'pid_denominator': 1000,
        #'pid_moving_buffer_size': 40,
        'pid_velocity': 0,
        'fw_max_pwm': 300,
        'wheel_type': 'standard'
    },
    'velocity_controller': {
        'wheel_separation_multiplier': 1.0,
        'wheel_radius_multiplier': 1.0,
        'linear': {
            'x': {
                'has_velocity_limits': False,
                'max_velocity': 1.0,
                'has_acceleration_limits': True,
                'max_acceleration': 0.5
            }
        },
        'angular': {
            'z': {
                'has_velocity_limits': False,
                'max_velocity': 2.0,
                'has_acceleration_limits': True,
                'max_acceleration': 5.0
            }
        }
    },
    'force_time_sync' : True,
    'oled_display': {
        'controller': None
    }
}

def check(conf, default_conf):
    for key, value in default_conf.items():
        if key not in conf:
            conf[key] = copy.deepcopy(value)
        elif type(default_conf[key]) is dict:
            if type(conf[key]) is not dict:
                raise ValueError("Config parameter is not dict")
            check(conf[key], default_conf[key])
    return conf

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

            # check if all parameters in default_config are set
            # if not, set them as default
            check(conf, default_conf)
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

    extrinsics_file = '~/.ros/camera_info/extrinsics_%s.yaml' % conf['raspicam']['position']
    extrinsics_file = os.path.expanduser(extrinsics_file)
    if not os.path.isfile(extrinsics_file):
        extrinsics_file = '-'

    m_controller = conf['motor_controller'] # make it easier to access elements
    v_controller = conf['velocity_controller'] # make it easier to access elements
    v_controller_linear = conf['velocity_controller']['linear']['x'] # make it easier to access elements
    v_controller_angular = conf['velocity_controller']['angular']['z'] # make it easier to access elements
    # Ugly, but works
    # just passing argv doesn't work with launch arguments, so we assign sys.argv
    sys.argv = ["roslaunch", "magni_bringup", "core.launch",
                         "raspicam_mount:=%s" % conf['raspicam']['position'],
                         "sonars_installed:=%s" % conf['sonars'],
                         "camera_extrinsics_file:=%s" % extrinsics_file,
                         "controller_serial_port:=%s" % m_controller['serial_port'],
                        #  "controller_serial_baud:=%s" % m_controller['serial_baud'],
                         "controller_pid_proportional:=%s" % m_controller['pid_proportional'],
                         "controller_pid_integral:=%s" % m_controller['pid_integral'],
                         "controller_pid_derivative:=%s" % m_controller['pid_derivative'],
                         #"controller_pid_denominator:=%s" % m_controller['pid_denominator'],
                         #"controller_pid_moving_buffer_size:=%s" % m_controller['pid_moving_buffer_size'],
                         "controller_pid_velocity:=%s" % m_controller['pid_velocity'],
                         "controller_fw_max_pwm:=%s" % m_controller['fw_max_pwm'],
                         "controller_wheel_type:=%s" % m_controller['wheel_type'],
                         "v_controller_wheel_separation_multiplier:=%s" % v_controller['wheel_separation_multiplier'],
                         "v_controller_wheel_radius_multiplier:=%s" % v_controller['wheel_radius_multiplier'],
                         "v_controller_linear_x_has_velocity_limits:=%s" % v_controller_linear['has_velocity_limits'],
                         "v_controller_linear_x_max_velocity:=%s" % v_controller_linear['max_velocity'],
                         "v_controller_linear_x_has_acceleration_limits:=%s" % v_controller_linear['has_acceleration_limits'],
                         "v_controller_linear_x_max_acceleration:=%s" % v_controller_linear['max_acceleration'],
                         "v_controller_angular_z_has_velocity_limits:=%s" % v_controller_angular['has_velocity_limits'],
                         "v_controller_angular_z_max_velocity:=%s" % v_controller_angular['max_velocity'],
                         "v_controller_angular_z_has_acceleration_limits:=%s" % v_controller_angular['has_acceleration_limits'],
                        #  "v_controller_angular_z_max_acceleration:=%s" % v_controller_angular['max_acceleration'],
                         "oled_display:=%s" % oled_display_installed
               ]


    print("------------------------------")
    print(sys.argv)
    roslaunch.main(sys.argv)

