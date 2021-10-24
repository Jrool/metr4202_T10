# metr4202-T10

Welcome Team 10 to the Git for the Project!\
Team Members + Current Roles:
- Adam
- John
- Josh
- Michael
- Tom

#Required ROS packages to work on this project

-ROS Noetic install:\
-moveit:\
    Install on Ubuntu 20.04: `sudo apt install ros-noetic-moveit`

#RECENT UPDATES:
Changed name of main branch to master\
`git branch -m main master`\
`git fetch origin`\
`git branch -u origin/master master`\
`git remote set-head origin -a`

#Robot Operation:
The Robot utilises the equipment covered in the project specificiation, and must be present and integrated within the raspberry pi in order to 
complete the project. The code can be launched using the following lines:
- `roslaunch physical_control program_setup.launch` => This launches the relevant supporting code for the Dynamaxil servos, SG90 servo, ximea_camera and the Rviz simulation
- `rosrun phsyical_control michael_boss.py`\ => This resets the robots configuration to the home position, and waits for user input to start the search loop
- `rosrun physical_control tf_broadcast.py`\ => This calibrates the based on the afixed fiduical marker 42, which allows the robot to identify the aruco tags in terms of the space frame. This is critical as it is essential in allowing the robot to compute accurate inverse kinematics solutions
- `rosrun colour_detection colour_detect.py`\ => Enables the colour detection topics used within the main program `michael_boss.py`