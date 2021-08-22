Garden Protector
![alt text](https://github.com/Nick-Manglaviti/Garden-Protector/Front_Image.jpg?raw=true)
![alt text](https://github.com/[username]/[reponame]/blob/[branch]/Back_Image.jpg?raw=true)
What Does this Robot Do?
	This sentry-like robot will scan its field of via the connected webcam and will detect any pesky squirrels 
	within is vision. Once detected, the robot will point itself directly at the squirrel and spray it with
	water until it leaves, thus protecting your plants or bird feeders from the unwanted guest (unless the squirrel
	is thirsty!).

Purpose For the Project
	The main purpose of this project was to get some hands-on experience designing and constructing a robot
	from conception to a finished product. I've been learning robotics by developing and learning through 
	an online ROS course, but most of that comes from the software perspective. Being a computer science, I've 
	also never had much experience with working on mechanical/electrical parts so being able to dive in and 
	create my very own robot was a good way to familiarize myself with the hardware construction and usage. 
	This is why the robot is more of a working concept rather than a real practical creation.

Install and Run
	The installation process involves some pre-prerequisite installations such as...
	1 - Ubuntu Mate (20.04) on a Raspberry Pi 4 Model B
	2 - ROS-Noetic with catkin_ws
	You can find many guides on how to install the above, so I will walk through as though you are a
    user who has some experience with ROS and is comfortable running commands from the command line on Linux machines.
	So, assuming the above is done, you would need to make sure the servos and camera are properly setup. I recommend 
	reading the Circuitry Section for a look at how I setup the wires and connections.
	1) Import the project to your catkin_ws/src directory.
	2) In catkin_ws/
		Run Command: catkin_make
		Run Command: source devel/setup.bash
	1) For the servos, makes sure the correct pins are setup in scripts/mappings.py.
	2) If those are good, run tests/servo_test.py and check that each servo moves and the order is correct. 
		In garden_protector/tests
		Run Command: python3 servo_test.py
		*NOTE: If you get a "Not running on a RPi" you may need to allow permissions in /dev/gpio*.
	3) pip install opencv-python if not already installed as well as any other python modules if any are missing.
	4) For the camera, run tests/camera_test.py. If you see your camera's feed pop-up then your good.
		In garden_protector/tests/ 
			Run Command: python3 camera_test.py
	*NOTE: If you used ssh to program/use your Raspberry Pi, then this will not work as it needs a display 
	to run (use remote desktop or hdmi to a monitor).
	5) If all went well, you can run the main launch file with the command... 
		roslaunch garden_protector active_detection.launch
	6) There are some ros.params in the launch file if you want to affect the behaviour a bit.

The Software

	For the Raspberry Pi 
		+ Ubuntu Mate (20.04 Focal Fossa)
		+ ROS Noetic
		+ Python3.8
		+ Remmina
	On my main Windows 10 Desktop
		+ TensorFlow 2 v2.5
			+ cuDNN 8.1
			+ CUDA 11.2
		+ labelimg Github Repo
		+ GenerateTFRecord Github Repo
	
	Description
		I installed Ubuntu Mate (20.04 Focal Fossa (1)) on the Pi's SD Card so that I could use ROS-Noetic
		as that is currently the officially supported OS for ROS (Robot Operating System (2)). The code for the robot 
		is all written in Python3.8. Remmina was used to Remote Desktop into the Pi, allowing me to stream the video
		using OpenCv. For the computer vision aspect of the program, I used an SSD MobileNet V2 FPNLite 320x320 
		which is one of the many available models at the TensorFlow 2 Detection Model Zoo (3). In order to speed 
		up the training process, I installed the proper versions of cuDNN (4) and CUDA (5) for GPU processing.
	    You need a Nvidia Graphics Card in order to use CUDA and cuDNN for training. Labelimg is a Github 
		Repo by the user tzutalin which allowed me to label and annotate images to be trained by the TensorFlow model 
		(6). GenerateTFRecord is a repo by the user nicknochnack which has a script that turns the test and train 
		images into a TFRecord, which can be inputed into the TensorFlow model(6). 
	
	Links (#)
		(1) https://ubuntu-mate.org/download/arm64/focal/
		(2)	http://wiki.ros.org/noetic
		(3) https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md
		(4)	https://developer.nvidia.com/cuda-11.2.0-download-archive
		(5)	https://developer.nvidia.com/cudnn
		(6) https://github.com/tzutalin/labelImg
		(7) https://github.com/nicknochnack/GenerateTFRecord

The Hardware

	Main Components
		1 Raspberry Pi 4 Model B
		1 4 AA Battery Holder
		1 Mini Breadboard
		3 MG996R 55g Servo Motors
		1 USB WebCamera
		1 Re-purposed Bottle Sprayer with Stream option
		1 Tubing for spray bottle
		1 Water Container
		1 5V Power Suppy Adapter to USB C
		1 Servo Mount Bracket Set, Steering Gear Pan and Tilt Mount
	
	Secondary Components
		2 Zip-Ties
		6 Male to Female Dupont Wires
		6 Male to Male Dupont Wires
		2 3D Printed PLA Platforms

	Optional Components
		1 RPi UPSPack Standard V3P
		1 Lipo Rechargeable Battery w/ JST Connector
		1 (USB A to USB C) Cable
		- Optional components just allow the robot to be mobile.
		
The Circuitry
	
		Circuitry on Mini Breadboard
			1st Row (+ Power Row)
				(1) Male to Female going to (+) on Battery Holder
				(2) Male to Male going to (+) on Servo1
				(3) Male to Male going to (+) on Servo2
				(4) Male to Male going to (+) on Servo3
			2nd Row (- Ground Row)
				(1) Male to Female going to (-) on Battery Holder
				(2) Male to Female going to (GND) on Raspberry Pi
				(3) Male to Male going to (-) on Servo1
				(4) Male to Male going to (-) on Servo2
				(5) Male to Male going to (-) on Servo3
				
		Circuitry on Raspberry Pi (referred by GPIO.BOARD, i.e. the physical # pin)
			Pin 6: Female to Male going to mini BB (Row 2, slot 2)
			Pin 7: Female to Male going to SIGNAL on Servo1
			Pin 8: Female to Male going to SIGNAL on Servo2
			Pin 11: Female to Male going to SIGNAL on Servo3
	
		*Note: If you swap or change the the SIGNAL pins, you can just edit
		in scripts/mappings.py the Servos(Enum) to the newly chosen pins. These
		are referenced by their Broadcom SOC channel number i.e. for physical pin 7
		it would be 4 for GPIO4.
	
The Assembly
	For the assembly, I just 3D printed 2 bases for the sentry bot. The first base just holds
	the Raspberry Pi, Battery Holder, and Mini Breadboard. There are 4 holes to lift and mount the pi
	slighty above the base so that the Pi is not resting on the plate. The first servo is bolted into the
	base plate with the Stearing Gear Pan connected to it. The next servo is bolted to the first with the tilt
	mount connected to its gear. The last servo is bolted in the second base plate. On the second base plate 
	is the trigger servo, bottle spray head, and webcamera. The webcamera and bottle spray head are held with 
	glue and an indentation slot for them to rest in. A zip-tie goes around a screw connected to the rotary pan 
	of the trigger servo to pull on the bottle spray head's trigger. 





