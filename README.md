# init_trial
mavros_init_trial
You should launch the minimum snap first to gengerate the data.txt in the right directory,
then, launch the script for aggressive flight!
Because of the failure in installing the mavros on raspberry 4B ,so I move to the python file. In the next, you should launch the gazebo by "make px4_sitl gazebo" in the project/Firmware, and launch the C++ file , which collect the fake_gps signal and send to the python. At last ,run the app.py to test the control method
Now, this code can fly by the eight shape circle using the genometric controller at the high speed!
