#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from  geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations
import math
import threading
import numpy as np
import matplotlib.pyplot as plt


def odom_callback(msg):
    lock.acquire()

    global exact_pose
    global is_exact_init
    exact_pose = msg.pose.pose
    is_exact_init = True

    lock.release()


def update_monitor():

    global is_exact_init

    if not is_exact_init:
        return

    # get SLAM estimated pose from transform
    try:
        tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(3))
        (slam_trans, slam_rot) = tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    slam_pose = Pose()
    slam_pose.position.x = slam_trans[0]
    slam_pose.position.y = slam_trans[1]
    slam_pose.position.z = slam_trans[2]
    slam_pose.orientation.x = slam_rot[0]
    slam_pose.orientation.y = slam_rot[1]
    slam_pose.orientation.z = slam_rot[2]
    slam_pose.orientation.w = slam_rot[3]
    slam_roll,slam_pitch,slam_yaw = tf.transformations.euler_from_quaternion(slam_rot)
    rospy.loginfo('\n---------------------------------------------')
    rospy.loginfo('\nSLAM pose: ' + str(slam_pose))

    # global vars
    lock.acquire()

    global exact_pose
    global samples
    global error_accum_x
    global error_accum_y
    global error_accum_z
    global error_accum_magnitude
    global error_accum_yaw
    global data_time
    global data_error_x
    global data_error_y
    global data_error_z
    global data_error_magnitude
    global data_error_roll
    global data_error_pitch
    global data_error_yaw
    global data_dev_x
    global data_dev_y
    global data_dev_z
    global data_dev_magnitude
    global data_dev_yaw
    global next_plot_time

    # get exact pose from exact Odometry topic
    exact_roll,exact_pitch,exact_yaw = tf.transformations.euler_from_quaternion(
        [exact_pose.orientation.x, exact_pose.orientation.y, exact_pose.orientation.z, exact_pose.orientation.w])
    rospy.loginfo('\nExact pose: ' + str(exact_pose))

    # calculate errors
    error_x = slam_pose.position.x - exact_pose.position.x
    error_y = slam_pose.position.y - exact_pose.position.y
    error_z = slam_pose.position.z - exact_pose.position.z
    error_magnitude = math.sqrt(error_x*error_x + error_y*error_y + error_z*error_z)
    error_roll = slam_roll - exact_roll
    error_pitch = slam_pitch - exact_pitch
    error_yaw = slam_yaw - exact_yaw

    # calculate standard deviations
    samples += 1
    error_accum_x += error_x * error_x
    error_accum_y += error_y * error_y
    error_accum_z += error_z * error_z
    error_accum_magnitude += error_magnitude * error_magnitude
    error_accum_yaw += error_yaw * error_yaw

    sdev_x = math.sqrt(error_accum_x / samples)
    sdev_y = math.sqrt(error_accum_y / samples)
    sdev_z = math.sqrt(error_accum_z / samples)
    sdev_magnitude = math.sqrt(error_accum_magnitude / samples)
    sdev_yaw = math.sqrt(error_accum_yaw / samples)

    lock.release()

    # errors data
    data_time = np.append(data_time, rospy.Time.now().to_sec())
    data_error_x = np.append(data_error_x, error_x)
    data_error_y = np.append(data_error_y, error_y)
    data_error_z = np.append(data_error_z, error_z)
    data_error_magnitude = np.append(data_error_magnitude, error_magnitude)
    # angle errors
    data_error_roll = np.append(data_error_roll, error_roll)
    data_error_pitch = np.append(data_error_pitch, error_pitch)
    data_error_yaw = np.append(data_error_yaw, error_yaw)
    # standard deviations
    data_dev_x = np.append(data_dev_x, sdev_x)
    data_dev_y = np.append(data_dev_y, sdev_y)
    data_dev_z = np.append(data_dev_z, sdev_z)
    data_dev_magnitude = np.append(data_dev_magnitude, sdev_magnitude)
    data_dev_yaw = np.append(data_dev_yaw, sdev_yaw)

    if samples == 1 or rospy.Time.now() >= next_plot_time:
        next_plot_time = rospy.Time.now() + rospy.Duration(1)
        # print results
        printErrorData()
        # draw plots
        error_pos_plot.clear()
        error_pos_plot.title.set_text("Positin errors: X, Y, Z, Magnitude")
        error_pos_plot.plot(data_time, data_error_x, 'r-')
        error_pos_plot.plot(data_time, data_error_y, 'g-')
        error_pos_plot.plot(data_time, data_error_z, 'b-')
        error_pos_plot.plot(data_time, data_error_magnitude, 'm-')
        error_angle_plot.clear()
        error_angle_plot.title.set_text("Rotation errors: Roll, Pitch, Yaw")
        error_angle_plot.plot(data_time, data_error_roll, 'r-')
        error_angle_plot.plot(data_time, data_error_pitch, 'g-')
        error_angle_plot.plot(data_time, data_error_yaw, 'b-')
        deviation_plot.clear()
        deviation_plot.title.set_text("Standard deviations: X, Y, Z, Magnitude")
        deviation_plot.plot(data_time, data_dev_x, 'r-')
        deviation_plot.plot(data_time, data_dev_y, 'g-')
        deviation_plot.plot(data_time, data_dev_z, 'b-')
        deviation_plot.plot(data_time, data_dev_magnitude, 'm-')
        # histograms
        pos_hist.clear()
        pos_hist.title.set_text("Position error: X")
        pos_hist.hist(data_error_x, bins='auto')
        pos_magnitude_hist.clear()
        pos_magnitude_hist.title.set_text("Position error: Magnitude")
        pos_magnitude_hist.hist(data_error_magnitude, bins='auto')

        fig.canvas.draw()
        fig.canvas.flush_events()


def printErrorData():
    rospy.loginfo("\n====================\nLocalization data:" +
            "\nPosition errors - Magnitude: " + str(data_error_magnitude[-1]) + ", X: " + str(data_error_x[-1]) +
                  ", Y: " + str(data_error_y[-1]) + ", Z: " + str(data_error_z[-1]) +
            "\nRotation errors - Roll: " + str(data_error_roll[-1]) + ", Pitch: " + str(data_error_pitch[-1]) +
                  ", Yaw: " + str(data_error_yaw[-1]) +
            "\nStandard deviations - Magnitude: " + str(data_dev_magnitude[-1]) + ", X: " + str(data_dev_x[-1]) +
                  ", Y: " + str(data_dev_y[-1]) + ", Z: " + str(data_dev_z[-1]) +
            "\n====================")


if __name__ == '__main__':

    global exact_pose
    global is_exact_init
    global samples
    global error_accum_x
    global error_accum_y
    global error_accum_z
    global error_accum_magnitude
    global error_accum_yaw
    global data_time
    global data_error_x
    global data_error_y
    global data_error_z
    global data_error_magnitude
    global data_error_roll
    global data_error_pitch
    global data_error_yaw
    global data_dev_x
    global data_dev_y
    global data_dev_z
    global data_dev_magnitude
    global data_dev_yaw

    lock = threading.Lock()
    exact_pose = Odometry()
    is_exact_init = False

    samples = 0
    error_accum_x = 0.0
    error_accum_y = 0.0
    error_accum_z = 0.0
    error_accum_magnitude = 0.0
    error_accum_yaw = 0.0

    data_time = np.array([])
    data_error_x = np.array([])
    data_error_y = np.array([])
    data_error_z = np.array([])
    data_error_magnitude = np.array([])
    data_error_roll = np.array([])
    data_error_pitch = np.array([])
    data_error_yaw = np.array([])
    data_dev_x = np.array([])
    data_dev_y = np.array([])
    data_dev_z = np.array([])
    data_dev_magnitude = np.array([])
    data_dev_yaw = np.array([])

    rospy.init_node('localization_monitor', anonymous=True)
    tf_listener = tf.TransformListener()
    rospy.Subscriber('/exact_pose', Odometry, odom_callback)

    plt.ion()
    fig = plt.figure()
    fig.canvas.set_window_title('Localization monitor')
    error_pos_plot = fig.add_subplot(321)
    error_angle_plot = fig.add_subplot(323)
    deviation_plot = fig.add_subplot(325)
    pos_hist = fig.add_subplot(322)
    pos_magnitude_hist = fig.add_subplot(324)
    fig.show()

    # print errors data on exit
    rospy.on_shutdown(printErrorData)
    # run update loop
    r = rospy.Rate(5)  # 5hz
    while not rospy.is_shutdown():
        update_monitor()
        r.sleep()
