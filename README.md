# ROS Course ETH 2020
day 1:
add teleop_twist_keyboard to your git folder (either by download from GitHub and moving it to the git folder in your explorer or by using git clone <...> in the command line
use the launch file called teleop_twist_keyboard.launch and start it from your src directory in the terminal using roslaunch teleop_twist_keyboard.launch (Make sure you add the path to the ~/git/teleop_twist_keyboard file as explained in the lecture.)

day 2:
Download the zip file from the ROS page and unpack it to get started.
subscribe to the /scan topic: 
  - add the 2 lines in my config.file, to the raw config file from the zip file. 
  - create a subscriber in the hpp file, and initialise it in the cpp file
create a callback function
  - create the function in the hpp file, and initialise it in the cpp file so it computes the desired distance. 
the new launch file is in the launch folder. I then created another launch file call ultimats_launcher which is located in the catkin_ws while the package itself is located in the git folder.
