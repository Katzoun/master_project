## DONT USE SUDO WITH BUILD SCRIPTS !!


# nepouzivat defaultni colcon pro build, pouze build script.


co je vlastne zatim potreba?
- nainstalovany ros2 humble
- PhoxiControl 1.15.0
- tmux



ros2 service call /capture_image interface_pkg/srv/CaptureImage "{transform: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16], camera_settings: 'test_settings'}"

uzitecne prikazy

pip list --local --format=freeze > requirements.txt
pip list --local

tmux kill-session -t ros_build


https://www.automate.org/robotics/industry-insights/ros-industrial-for-real-world-solutions
https://rosindustrial.org/scan-n-plan