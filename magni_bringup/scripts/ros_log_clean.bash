#!/bin/bash

# Removes logs out of a ros directory older then x days. Delets the whole log directory if exceeds specified size.
# Usage: Simply run it with "source <path_to_scrpit>/ros_log_clean.sh" from terminal or if you want it automated from .bashrc OR /usr/sbin/magni-base

# Number of days after log files will be removed (set without any spaces)
days_to_remove=2

# Size limit in bytes on the ~/.ros/log/ folder. If this limit reached, whole log folder is deleted
logdir_size_limit=2000000

if [ ! -d ~/.ros/log/ ]; then
    echo "ROS_LOG_CLEANER: Directory ~/.ros/log/ does not exist."
    return 0
fi

# Delete files that have not been modified in $days_to_remove days
removed_file_counter=0
for i in $( find ~/.ros/log/ -ctime +${days_to_remove} -type f | grep \.log ); do
    #echo $i" unmodified for (days): "$days_to_remove
    rm $i
    let removed_file_counter=removed_file_counter+1
done
echo "ROS_LOG_CLEANER: Removed "$removed_file_counter" ROS log files unmodified for longer then "$days_to_remove" days"

# Delete files older than a $days_to_remove days
removed_file_counter=0
for i in $( find ~/.ros/log/ -mtime +${days_to_remove} -type f | grep \.log ); do
    #echo "older then (days) "$days_to_remove
    rm $i
    let removed_file_counter=removed_file_counter+1
done
echo "ROS_LOG_CLEANER: Removed "$removed_file_counter" ROS log files older then "$days_to_remove" days"

# Delete empty directories
removed_file_counter=0
for i in $( find ~/.ros/log/ -type d -empty); do
    #echo $i
    rm -rf $i
    let removed_file_counter=removed_file_counter+1
done
echo "ROS_LOG_CLEANER: Removed "$removed_file_counter" empty ROS log directories"

# Delete the whole log directory if it exceeds specified size
log_dir_size=$(du -s ~/.ros/log/ | cut -f1)
echo "ROS_LOG_CLEANER: Log directory size: "$log_dir_size" bytes"
if (( log_dir_size > logdir_size_limit )) ; then
    echo "ROS_LOG_CLEANER: Log directory larger then the logfile size limit: "$logdir_size_limit" bytes. Deleting log dir."
    rm -rf ~/.ros/log/
fi
