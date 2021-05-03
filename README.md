# Installation of caffe 

I mostly followed the instructions from this page: https://www.programmersought.com/article/99264953862/
Follow the instructions exactly until you have built caffe and you shouldn't run into any problems. For the python installation, refer
back to this README. Make sure to use my .zip of caffe
as your master directory, as I have edited some of the source code to make it compatible with the version of openpose
used in this project.

Start by unzipping the caffe, openpose, and project zip files to your home directory.


After you've built caffe
Next install pip for python using the following commands.

`curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get_pip.py`

`sudo python get_pip.py`

`python get_pip.py` for good measure.

You may need to install curl with `sudo apt install curl`

Next navigate to the python directory in caffe and run the following command.

`for req in $(cat requirements.txt);do sudo pip install $req;done`

and `pip install -r requirements.txt` for good measure.

All modifications for your specific build of caffe should be made to `Makefile.config`

Next in `~/caffe` run `sudo make clean` and then `make all`

If there are errors, make sure all of your paths are correct on `Makefile.config`. 
I've modified it to work for most of the ros paths, but sometimes numpy gets installed
in different locations depending if you installed it using `sudo pip` or just `pip`. 

Finally, run `make pycaffe`
Make sure you can import caffe in python.


# Intallation of openpose

`git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose`
`cd openpose`
`mkdir build`
`cd build`
`sudo make -j$(nproc) && sudo make install -j$(nproc)`

navigate to the build directory and run this command (you may need to install cmake):
`cmake -DGPU_MODE=CPU_ONLY  -DUSE_MKL=OFF -DCaffe_INCLUDE_DIRS=~/caffe/include/ 
-DCaffe_LIBS=~/caffe/build/lib/libcaffe.so -DBUILD_CAFFE=OFF -DBUILD_PYTHON=ON 
-DPYTHON_LIBRARY=$(python-config --prefix)/lib/libpython2.7.dylib 
-DPYTHON_INCLUDE_DIR=$(python-config --prefix)/include/python2.7  -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python ..`

Import it as such:
```
import sys
sys.path.append('/usr/local/python')
from openpose import pyopenpose as op
```

I also ran into some troubles where the openpose library files were not on `LD_LIBRARY_PATH`
Use `locate` to locate any library files it cannot find, and make sure those paths get added to `LD_LIBRARY_PATH`.
I personally had trouble where `/usr/local/lib` did not end up on my `LD_LIBRARY_PATH`

# Additional libraries to install for python
`pip install pandas`
`pip install pathlib`

# Installation of universal robot
Follow instructions here for installation: https://github.com/ros-industrial/universal_robot

# Running

So gazebo works properly, execute `echo "export SVGA_VGPU10=0" >> ~/.profile && source ~/.profile`
You will need to initialize your own catkin ws and clone my repo into the `src` directory and run `catkin_make` back in `~/catkin_ws`
Next, in `~/catkin_ws`, do `mkdir -p assets/frames`

Follow instructions here for installation of the Universal Robot: https://github.com/ros-industrial/universal_robot
You will need to make one change in order for gazebo to run stably on 16.04. 
`cd ~/catkin_ws/src/ur_description/urdf`
In the editor of your choice, add the following line inside the `<plugin>` tag of the .gazebo file: `<legacyModeNS>true</legacyModeNS>`

The resulting file should look something like this:
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

  <gazebo>
    <plugin name="ros_control" filename="libgazebo_ros_control.so">
      <legacyModeNS>true</legacyModeNS>
      <!--robotNamespace>/</robotNamespace-->
      <!--robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType-->
    </plugin>

<!--
    <plugin name="gazebo_ros_power_monitor_controller" filename="libgazebo_ros_power_monitor.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1.0</updateRate>
      <timeout>5</timeout>
      <powerStateTopic>power_state</powerStateTopic>
      <powerStateRate>10.0</powerStateRate>
      <fullChargeCapacity>87.78</fullChargeCapacity>     
      <dischargeRate>-474</dischargeRate>
      <chargeRate>525</chargeRate>
      <dischargeVoltage>15.52</dischargeVoltage>
      <chargeVoltage>16.41</chargeVoltage>
    </plugin>
-->
  </gazebo>

</robot>
```

Next, from inside this repo, `cp -r scripts ~/catkin_ws/src/universal_robot/ur_gazebo/` 
This will put the scripts in the ur_gazebo workspace. In `~/catkin_ws` run `catkin_make`

Make sure you source it `source ~/catkin_ws/devel/setup.bash`

Next, you will need to edit one of the scripts to make sure it write to your home directory
Navigate to `~/catkin_ws/src/universal_robot/ur_gazebo/scripts`
Open `pre_process_video.py` and change `HOME_DIR` to your fully qualified home directory.

Be sure to do the same in `find_hand.py`, otherwise it will not be able to find the openpose models.

Now all you need to do is run `pre_process_video.py /path/to/video num_frames` where num frames is how many frames you want it to grab from the video.
Note that the frames take out are evenly selected from the films. 

This might take awhile, but the results are really cool and worth it.

To run the simulation run `roslaunch ur_gazebo ur10.launch`
The IK node is not implemented on any of the other launch files, but you can do this by adding a single line to the launch file.
I haven't tested this IK solver with any of the other UR robots, but my strong gut feeling is that it will only work for the UR10.

Here are some example calls
`python pre_process_video /home/espen/hand_video_1.mpeg 32`
`roslaunch ur_gazebo ur10.launch`
`rosrun ur_gazebo send_poses_preprocessed.py -p -n 32`

Info on the scripts
`send_poses_pre_processed.py`
```
A script to save frames and hand centerpoints from a video of a hand

optional arguments:
  -h, --help            show this help message and exit
  -n NUM_FRAMES, --num_frames NUM_FRAMES
                        How many frames you want to process. You must have
                        already pre processed exactly this many frames
  -p, --pause_bt_frames
                        Pass this argument if you wish to have control of how
                        long you stay at each frame. for each picture
                        displayed, just press any key to advance to the next
                        frame and observe how the robot moves from frame to
                        frame.
  -rt, --real_time      This argument will cause the script the script to take
                        into account the timestamps from each frame, and delay                                                                                                                                                                                        
                        for the appropriate amount of time before sending the                                                                                                                                                                                         
                        next message.      
                        frame.
```

`pre_process_video.py`
```
A script to save frames and hand centerpoints from a video of a hand

positional arguments:
  path        Path to video file
  num_frames  However many frames you wish to extract

optional arguments:
  -h, --help  show this help message and exit
```













