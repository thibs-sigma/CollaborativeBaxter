# CollaborativeBaxter

## Repo for KTH - FACT Project demonstrator of collaborative operations

- Pickup object
- Assembly task
- Inspection task

## Launch

The easiest way is to use TeamViewer (add credentials + trick if black/small/slow screen)
Open a terminal:
- Terminal 1:
  - `ssh administrator@cpr-ridgeback`
  - Enable bluetooth for teleop (if required): `sudo hciconfig hci0 up`
  - Start screen session: `screen` (for letting it running in background)
  - Screen 1:
    - `source /opt/ros/indigo/setup.bash && export ROS_MASTER_URI=http://011509P0021.local:11311`
    - `sudo ros service stop` (pass: clearpath)
    - `roslaunch ridgeback_base base.launch`
  - Screen 2: (Ctrl+A then `c`)
    - `source /opt/ros/indigo/setup.bash && export ROS_MASTER_URI=http://011509P0021.local:11311`
    - `roslaunch ridgeback_bringup accessories.launch`
  - Screen 3: (Ctrl+A then `c`)
    - Detach screen: `screen -d`
- Terminal 2 (new tab or window):
  - `roslaunch ridgeback_baxter_description description.launch`
- Terminal 3 (new tab or window):
  - Check if RealSense cameras are detected `rs-enumerate-devices | grep Serial`
  - `roslaunch realsense2_camera launch_realsense_intel_multi.launch`
- Terminal 4 (new tab or window): 
  - Navigation: `roslaunch rtabmap_ros ridgeback_multi_navigation.launch`
  - Mapping: `roslaunch rtabmap_ros ridgeback_multi_mapping.launch` (**explain how to edit database filename**)
- Terminal 5 (new tab or window):
  - State Machine: `roslaunch smach_baxter launch_baxter_manipulation.launch`
  

   
