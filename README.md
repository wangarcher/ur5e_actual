# ur5e_actual
ur5e_actual

### 1. Requirements
System: Ubuntu 18.04, Ubuntu 20.04   

ROS: Melodic, Noetic    
 
Moveit   

   

### 2. Other Dependencies
For some dependencies were missed for the project, please check the following instructions.
```
sudo apt-get install ros-melodic-moveit-visual-tools
sudo apt-get install ros-melodic-ros-controllers
sudo apt-get install ros-melodic-ur-client-library
sudo apt-get install ros-melodic-ur-msgs
```

**NOTE:**There might be more dependencies needed due to vairous reasons. If you found any, please let me know.(archer7wang@outlook.com)

### 3. Usage
Like ordinary ros projects, the simluation needs a clean workspace.
```
mkdir -p ur5e_actual_ws/src
cd src
git clone https://github.com/wangarcher/ur5e_actual.git
cd ..
catkin_make
source devel/setup.bash
``` 
##### 0. Power up the UR5e robot
Press the power bottom in the UR5e robot panel. And wait, then power on the robot by clicking the red icon in the left bottom.


##### 1. Initialization
To connect the UR5e robot with the PC. 
```
roslaunch ur_robot_driver ur5e_bringup_with_ip.launch
```

##### 2. Start the interface
To run the interface
```
roslaunch ur5e_move_group_interface run_arm.launch
```

##### 3. Plan
To run the project planning node
```
roslaunch ur5e_plan ur5e_project_plan.launch
```

##### 4. The working follow chart
```mermaid
    flowchart TD;
    start-->pre_pick;
    pre_pick-->get_pick_pose;
    get_pick_pose-->pick;
    pick-->pre_place;
    pre_place-->get_place_pose;
    get_place_pose-->place;
    place-->end;

```
```
start:          Init
pre_pick:       Moving to 1st reco configuration
get_pick_pose:  Get the object pose
pick:           Moving towards to that pose then pick
pre_place:      Moving to 2nd reco configuration
get_place_pose: Get the target pose
place:          Moving towards to that pose then place
end:            Done
```
Be advised! The pipeline is logical unfriendly. Any modification of the working flow might be devastated!

After moving to the pre pick pose, the robot would send a std_msgs::Int8 msg with data "1". And after moving to the pre place pose, the robot would send a std_msgs::Int8 msg with data "2". The reco module can then be enabled after such trigger is sent. The topic name of the trigger msg is "/reco_trigger"

And the reco module would feedback with a geometry_msgs::PoseStamped msg.
The msg would provide the target pose according to the camera reference frame.
For the picking object, the "frame_id" of the msg shall be write as "1st".
And the for the placing target, the "frame_id" of the msg shall be write as "2nd".