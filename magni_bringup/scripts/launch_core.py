#!/usr/bin/python

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

import sys, os, subprocess, time
import roslaunch, rospkg
import yaml
import smbus # used for the hw rev stuff

conf_path = "/etc/ubiquity/robot.yaml"

default_conf = \
{
    'raspicam' : {'camera_installed' : 'True', 'position' : 'upward'},
    'lidar' : {'lidar_installed' : 'True', 'position' : 'top_plate'},
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
    },

    'ubiquity_velocity_controller' : {
        'wheel_separation_multiplier': 1.0, # default: 1.0
        'wheel_radius_multiplier'    : 1.0, # default: 1.0    
        'linear':{
            'x':{
                'has_velocity_limits'    : 'True',
                'max_velocity'           : 1.0,   # m/s
                'has_acceleration_limits': 'True',
                'max_acceleration'       : 1.1,   # m/s^2
            }
        },
        'angular': {
            'z': {
                'has_velocity_limits'    : 'True',
                'max_velocity'           : 2.0,   # rad/s
                'has_acceleration_limits': 'True',
                'max_acceleration'       : 5.0,   # rad/s^2    
            }
        }
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

default_lidar_extrinsics = \
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
                y_conf = yaml.safe_load(conf_file)
            except Exception as e:
                print('Error reading yaml file, using default configuration')
                print(e)
                return default_yaml

            if (y_conf is None):
                print('WARN ' + path + ' is empty, using default configuration')
                return default_yaml

            for key, value in default_yaml.items():
                if key not in y_conf:
                    y_conf[key] = value

            return y_conf
    except IOError:
        print("WARN " + path + " doesn't exist, using default configuration")
        return default_conf
    except yaml.parser.ParserError:
        print("WARN failed to parse " + path + ", using default configuration")
        return default_conf

def create_core_launch_file(path,
                            conf = default_conf,
                            camera_extrinsics_file = "",
                            lidar_extrinsics_file = "",
                            oled_display = 0,
                            board_rev = 0):

    try:
        sonars_installed = conf['sonars']
        motor_controller_params = conf['motor_controller']
        # velocity controller params
        vc_params = conf['ubiquity_velocity_controller']
    except Exception as e:
        print ("There is an error with the conf: " + str(e))
        return False

    if len(motor_controller_params)!=9: 
        print("Motor_controller_params dictionaty must contain 9 items. Instead it has:", str(len(motor_controller_params)))
        return False

    # TODO should I be doing more checks here -> maybe if dictionary and such

    # create if not exsisting, overwrite stuff thats already there
    f = open(path, "w")
    f.write("<launch>\n")

    # add the robot description part
    f.write("""
    <include file="$(find magni_description)/launch/description.launch">
        <arg name="camera_extrinsics_file" value='""" +str(camera_extrinsics_file)+ """'/>
        <arg name="lidar_extrinsics_file" value='""" +str(lidar_extrinsics_file)+ """'/>
        <arg name="sonars_installed" value='""" +str(sonars_installed)+ """'/>
    </include>
    """)

    if bool(sonars_installed):
        f.write('\n\t<node pkg="pi_sonar" type="pi_sonar" name="pi_sonar"/>\n')
    else:
        f.write('\n\t<!--<node pkg="pi_sonar" type="pi_sonar" name="pi_sonar"/>-->\n')

    # oled_display adding if True, if False add it in anyway but commented out for easier debugging
    if oled_display:
        f.write('\n\t<node pkg="oled_display_node" type="oled_display_node" name="oled_display"/>\n')
    else:
        f.write('\n\t<!--<node pkg="oled_display_node" type="oled_display_node" name="oled_display"/>-->\n')


    f.write("""
    <node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostics_agg">
        <!-- Load the file you made above -->
        <rosparam command="load" file="$(find magni_bringup)/param/diagnostics_agg.yaml"/>
    </node>

    <!-- Load the parameters used by the following nodes -->
    <rosparam command="load" file="$(find magni_bringup)/param/base.yaml" />
    """)
    
    # board version adding if not 0, if zero add it in anyway but commented out for easier debugging
    if board_rev != 0:
        f.write('\n\t<param name="/ubiquity_motor/controller_board_version" value="'+str(board_rev)+'"/>\n')
    else:
        f.write('\n\t<!--<param name="/ubiquity_motor/controller_board_version" value="'+str(board_rev)+'"/>-->\n')

    # add PID Params
    f.write("""
    <param name="/ubiquity_motor/serial_port" value='"""+str(motor_controller_params['serial_port'])+"""'/>
    <param name="/ubiquity_motor/serial_baud" value='"""+str(motor_controller_params['serial_baud'])+"""'/>
    
    <!-- PID Params -->
    <param name="/ubiquity_motor/pid_proportional" value='"""+str(motor_controller_params['pid_proportional'])+"""'/>
    <param name="/ubiquity_motor/pid_integral" value='"""+str(motor_controller_params['pid_integral'])+"""'/>
    <param name="/ubiquity_motor/pid_derivative" value='"""+str(motor_controller_params['pid_derivative'])+"""'/>
    <param name="/ubiquity_motor/pid_denominator" value='"""+str(motor_controller_params['pid_denominator'])+"""'/>
    <param name="/ubiquity_motor/pid_moving_buffer_size" value='"""+str(motor_controller_params['pid_moving_buffer_size'])+"""'/>
    <param name="/ubiquity_motor/pid_velocity" value='"""+str(motor_controller_params['pid_velocity'])+"""'/>
    
    """)

    # add the roscontrol controller
    f.write("""
    <!-- Launch the roscontrol controllers needed -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner"
        args="ubiquity_velocity_controller ubiquity_joint_publisher"/>
    """)

    vc_lin = vc_params['linear']['x']
    vc_ang = vc_params['angular']['z']
    # add the motor node
    f.write("""
    <!-- Launch the motor node with the topic remapped to standard names -->
    <node name="motor_node" pkg="ubiquity_motor" type="motor_node">
        <remap from="/ubiquity_velocity_controller/cmd_vel" to="/cmd_vel"/>
        <remap from="/ubiquity_velocity_controller/odom" to="/odom"/>

        <param name="/ubiquity_velocity_controller/wheel_separation_multiplier" value='"""+str(vc_params['wheel_separation_multiplier'])+"""'/>
        <param name="/ubiquity_velocity_controller/wheel_radius_multiplier" value='"""+str(vc_params['wheel_radius_multiplier'])+"""'/>
        <param name="/ubiquity_velocity_controller/linear/x/has_velocity_limits" value='"""+str(vc_lin['has_velocity_limits'])+"""'/>
        <param name="/ubiquity_velocity_controller/linear/x/max_velocity" value='"""+str(vc_lin['max_velocity'])+"""'/>
        <param name="/ubiquity_velocity_controller/linear/x/has_acceleration_limits" value='"""+str(vc_lin['has_acceleration_limits'])+"""'/>
        <param name="/ubiquity_velocity_controller/linear/x/max_acceleration" value='"""+str(vc_lin['max_acceleration'])+"""'/>

        <param name="/ubiquity_velocity_controller/angular/z/has_velocity_limits" value='"""+str(vc_ang['has_velocity_limits'])+"""'/>
        <param name="/ubiquity_velocity_controller/angular/z/max_velocity" value='"""+str(vc_ang['max_velocity'])+"""'/>
        <param name="/ubiquity_velocity_controller/angular/z/has_acceleration_limits" value='"""+str(vc_ang['has_acceleration_limits'])+"""'/>
        <param name="/ubiquity_velocity_controller/angular/z/max_acceleration" value='"""+str(vc_ang['max_acceleration'])+"""'/>
    </node>
    
    """)

    f.write("</launch>")
    f.close()

    return True

# finds file path by priority:
# 1.) Checks first path and returns it if it exsists
# 2.) Checks second path and returns it if it exsists
# 3.) If none of them exsist returns empty string
def find_file_by_priority(first_path, second_path):
    first_path = os.path.expanduser(first_path)
    if not os.path.isfile(first_path):
        print("File "+first_path+" does not exsist")
        second_path = os.path.expanduser(second_path)
        if not os.path.isfile(second_path):
            print("File "+second_path+" does not exsist, using default config")
            return ""
        else:
            print("File "+second_path+" found")
            return second_path
    else:
        print("File "+first_path+" found")
        return first_path

if __name__ == "__main__":
    print (conf_path)
    conf = get_yaml(conf_path, default_conf)

    # We only support 1 version of the Sonars right now
    if conf['sonars'] == 'pi_sonar_v1':
        conf['sonars'] = True
    else:
        conf['sonars'] = False

    # We only support 1 display type right now
    oled_display_installed = False
    if conf['oled_display']['controller'] == 'SH1106':
        oled_display_installed = True

    # print (conf)

    if conf['force_time_sync']:
        time.sleep(5) # pifi doesn't like being called early in boot
        try:
            timeout = time.time() + 40 # up to 40 seconds
            while (1):
                if (time.time() > timeout): 
                    print ("Timed out")
                    raise RuntimeError # go to error handling
                output = subprocess.check_output(["pifi", "status"])
                if "not activated" in output:
                    time.sleep(5)
                if "acting as an Access Point" in output:
                    print ("we are in AP mode, don't wait for time")
                    break # dont bother with chrony in AP mode
                if "is connected to" in output:
                    print ("we are connected to a network, wait for time")
                    subprocess.call(["chronyc", "waitsync", "20"]) # Wait for chrony sync
                    break
        except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
            print ("Error calling pifi")
            subprocess.call(["chronyc", "waitsync", "6"]) # Wait up to 60 seconds for chrony
    else:
        print ("Skipping time sync steps due to configuration")

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
            print ("Got board rev: %d" % boardRev)
        except: 
            print ("Error reading motor controller board version from i2c\n")

    
    
    """
    check for extrinsics files in two places with following priorities:
        1.) in ~/.ros/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml
        2.) in package magni_description/param/<SENSOR>_extrinsics_<POSITION>.yaml
    If no file was found, do not load the sensor in urdf
    """
    rp = rospkg.RosPack()
    magni_description_path = rp.get_path('magni_description')

    # get camera extrinsics
    path1 = '~/.ros/extrinsics/camera_extrinsics_%s.yaml' % conf['raspicam']['position']
    path2 = magni_description_path+'/param/camera_extrinsics_%s.yaml' % conf['raspicam']['position']
    camera_extr_file = find_file_by_priority(path1, path2)
    if camera_extr_file == "":
        print ("Camera will not be enabled in urdf")
    else:
        print ("Camera enabled in urdf")


    # get lidar extrinsics
    path1 = '~/.ros/extrinsics/lidar_extrinsics_%s.yaml' % conf['lidar']['position']
    path2 = magni_description_path+'/param/lidar_extrinsics_%s.yaml' % conf['lidar']['position']
    lidar_extr_file = find_file_by_priority(path1, path2)
    if lidar_extr_file == "":
        print ("Lidar will not be enabled in urdf")
    else:
        print ("Lidar enabled in urdf")

   
    launch_file_path = "/tmp/core.launch" #TODO move this in /tmp/ - in home for debug
    create_success = create_core_launch_file(launch_file_path,
                                            conf = conf,
                                            camera_extrinsics_file=camera_extr_file,
                                            lidar_extrinsics_file=lidar_extr_file,
                                            oled_display = oled_display_installed,
                                            board_rev = boardRev)

    if not create_success:
        print("Creating launch file did not sucsseed")

    # TODO launch launch_file_path
    
