# init_trial
mavros_init_trial
You should launch the minimum snap first to gengerate the data.txt in the right directory,
then, launch the script for aggressive flight!

I change the method for the experiment, first, launch the gazebo by "make px4_sitl gazbeo",
then launch the C++ files in the package "gazebo_listener", which will publish the GPS message.
At last, using python3 to launch the script.
