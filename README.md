# CoachbotSwarm Repo

The Coachbot Swarm and this repo is owned and maintained by the Rubenstein Lab at Northwestern University. 
Check the user guide for more detailed information about how to use the system and troubleshooting: https://docs.google.com/document/d/1fQcU9cK_8bXtfPoXu4C7zOwC8WK4yg2npxJTBvi9Fgg/edit?usp=sharing

## Obtaining Contributer Access to the Repo

To gain access to push your code to the repo, email us at coachbotswarmsystem@gmail.com with your name, university, and github username and/or email address associated with the account.

## Submitting Code to be Run on the Coachbot System

Every submission to our system must be a folder with three files. The first is a .txt file called email.txt which simply contains the email address that should be contacted about the submitted code. The second file specifies the initial positions of the robots before the user code is run. Specifications of that file’s format and restrictions on robot positions are outlined in Section II.A.1. The third mandatory file is the code that will be uploaded to all active robots. This file, usr_code.py, must be written in python and formatted in the way outlined in Section II.A.2. The robots run python 2.7.16. There is a sample folder in the repo containing all three required files and a template for usr_code.py in the Example_Folder. Remember that your code should be placed in a folder in the Code_Queue folder to be run. 
Check the User Guide for more details on the guidelines of these input files.

usr_code.py format:
`import math
import struct

def usr(robot):
	# write code here
	while condition:
		robot.delay() # defaults to 20 ms which is sufficient
		# more code here
return condition`

## Available Robot Functions

1. robot.set_vel(left,right)
Parameters: left and right should be whole numbers between -50 and 50 that indicate wheel speeds. 0 being no movement and 50 being the fastest possible speed. The negative values indicate that the wheel should spin backwards at that speed. These values have a unit of 
Output: none
Example: robot.set_vel(30,-40)

2. robot.set_led(r,g,b)
Parameters: r,g,b should be integers between and 0 and 100 to set the color and brightness of the onboard LED
Output: none
Example: robot.set_led(30,100,0)

3. robot.virtual_id()
Parameters: none
Output: An integer that is the virtual ID of the robot. 
Example: virt_id = robot.virtual_id()

4. robot.get_clock()
Parameters: none
Output: a float of the number of seconds elapsed since the program started
Example: curr_time = robot.get_clock()

5. robot.send_msg(msg)
Parameters: msg should be a string that is less than 64 bytes or it will be truncated. This msg can be the output of the struck.pack() function explained in the section below.
Output: True is successful, False if not
Example: robot.send_msg(struct.pack(‘fffii’, float_0, float_1, float_2, int_0, int_1))

6. robot.recv_msg()
Parameters: none
Output: Returns the messages in the buffer since the last call of this function. 
Example: msgs = robot.recv_msg()

7. robot.get_pose()
Parameters: none
Output: a list with the [x,y,theta], check to see that this output is valid before using it
Example: pose = robot.get_pose()

8. robot.delay()
Parameters: default is 20ms but a different integer parameter can be specified
Output: none
Example: robot.delay(500)

## What to Expect From the Results

The results of your algorithm will be in the Completed_Runs folder of the repo. Once you pull the repo, you can navigate in to the Completed_Runs folder to your folder. If you forget the name of your folder, it will be sent in the email you received notifying you that the code was done running. Within this folder, you should see the following directory tree (new files and directories are in bold).
	OriginalName_completed
		usr_code.py
		init_poses.csv
		email.txt
		**init_pose_errors.csv**
		**output_logs**
			**ID_mapping.csv**
			**camera_video.mp4**
			**#_logging.csv**
			**#**
			**automation_errors**

The **init_pose_errors.csv** will contain the contents of **init_poses.csv** or list any issues with the initial poses specified. The output_logs folder will hold all the outputs from the algorithm run. Since the ID you specify in the **init_poses.csv** file might not match the physical robot ID, the **ID_mapping.csv** file specifies which robot corresponds to which virtual ID. The .mp4 file is the recording of the run from our overhead raspberry pi camera. The logging files will be named with the virtual ID of the robot it pertains to and contain the position of the corresponding robot at every timestep of the run. This csv file is formatted in a timestep, x position, y position, theta angle in radians for each line. The **#** file is the virtual ID of the pertaining robot and will have any information you choose to write to the **experiment_log** file in your code. The **automation_errors** file will list any high level errors such as runtime limits or robots trying to exit the play field.

## Simulation

Check out our Coachbot simulation here: https://github.com/michelleezhang/swarm_simulation/tree/master. The README of this repo contains detailed instructions on how to download and use this tool.
