To run:

Head to the root folder (The one having build, install, log, and src)
```
source install/setup.bash
```

```
ros2 launch handrobot_ros2_control handrobot.launch.py
```

(Now to run the controls)
```
ros2 run handrobot_ros2_control udp_data_reader
ros2 run handrobot_ros2_control camera_controller
```

(Copy RunAtWindows/mediapipe_hand/main.py to Windows and run it)
In Windows - where mediapipe_hand/main.py file is located:
```
python3 main.py
```
