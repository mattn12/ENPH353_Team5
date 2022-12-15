## Starting the sim
```
source ~/ros_ws/devel/setup.bash
cd ~/ros_ws/src/2022_competition/enph353/enph353_utils/scripts
./run_sim.sh
```

## Robot control script
Located in
`
~/ros_ws/src/2022_competition/adeept_awr/adeept_awr_gazebo/scripts
`

To run:
```
source ~/ros_ws/devel/setup.bash
rosrun enph353_comp_controller <controller code name>.py
```

## Score tracker
```
source ~/ros_ws/devel/setup.bash 
rosrun enph353_utils score_tracker.py 
```

## Location of License Plate png
`
~/ENPH353-Team5/ros_ws/src/2022_competition/enph353/enph353_gazebo/scripts/blank_plate.png
`

## Files to edit
potentially things in the adeept_awr and adeept_awr_gazebo directories
