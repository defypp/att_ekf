# att_ekf
Extented Kalman filter for attitude estimation using imu and magnetic filed.


# How to run the code
    cd catkin_ws/src
    git clone git@github.com:libing64/att_ekf.git
    cd ..
    catkin_make -DCATKIN_WHITELIST_PACKAGES="att_ekf"
    roslaunch att_ekf att_ekf.launch
#rosbag for test
1. download the rosbag

    https://drive.google.com/folderview?id=0B4hFvojO5r3sQ01WSHRkWFU2WVE&usp=sharing
    
2. replay the rosbag

    rosbag play 2016-02-29-19-05-57.bag

![att_ekf](https://cloud.githubusercontent.com/assets/3192355/13601467/57c2536e-e56b-11e5-82d5-25c8cbc9657f.png)

![att_ekf2](https://cloud.githubusercontent.com/assets/3192355/13601468/57edef42-e56b-11e5-927f-b453604b09f0.png)

    
# reference: 
https://pixhawk.org/_media/firmware/apps/attitude_estimator_ekf/ekf_excerptmasterthesis.pdf
http://web.cs.iastate.edu/~cs577/handouts/quaternion.pdf