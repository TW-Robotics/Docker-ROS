
#1 CALIBRATION:

roslaunch ur_calibration calibration_correction.launch robot_ip:=10.0.0.103 target_filename:="${HOME}/ur5.yaml"
roslaunch ur_calibration calibration_correction.launch robot_ip:=10.0.0.21 target_filename:="${HOME}/ur5.yaml"


#2 UR-DRIVER:

roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=10.0.0.103 kinematics_config:=${HOME}/ur5.yaml
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=10.0.0.21 kinematics_config:=${HOME}/ur5.yaml

#3 MOVEIT:

roslaunch ur5_moveit_config moveit_planning_execution.launch

#4 RVIZ:

roslaunch ur5_moveit_config moveit_rviz.launch


#### DATAcapturing TUWien/Bernhard

cd hemisphere_sampling/
python3 hemisphere_sampling_kuka.py 



# manual tansforms:
 rosrun tf static_transform_publisher -0.000854 0 0.029708 0 -2.35619 0 /tool0 /d435 100
 
 rosrun tf static_transform_publisher -0.000854 0 0.029708 0 -2.35619 0 /tool0 /camera_base_link 100
 
rosrun tf static_transform_publisher -0.000854 0 0.029708 0 -2.35619 0 /tool0 /camera_link 100



TERMINATOR WINDOS:
STRG+SHIFT + O 
STRG+SHIFT + E
STRG+SHIFT * D
