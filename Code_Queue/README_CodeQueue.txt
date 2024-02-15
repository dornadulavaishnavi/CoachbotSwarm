Full User Guide is located at: https://docs.google.com/document/d/1fQcU9cK_8bXtfPoXu4C7zOwC8WK4yg2npxJTBvi9Fgg/edit?usp=sharing

Submitting your Code

Every submission to our system must be a folder with three files. The first is a .txt file called email.txt which simply contains the email address that should be contacted about the submitted code. The second file specifies the initial positions of the robots before the user code is run. Specifications of that file’s format and restrictions on robot positions are outlined in Section II.A.1. The third mandatory file is the code that will be uploaded to all active robots. This file, usr_code.py, must be written in python and formatted in the way outlined in Section II.A.2. The robots run python 2.7.16. There is a sample folder in the repo containing all three required files and a template for usr_code.py in the Example_Folder. Remember that your code should be placed in a folder in the Code_Queue folder to be run.


Initial Positions
The initial positions of your robot must be specified in a csv file named init_poses.csv. The values specified must follow the rules listed below.
Each row should specify an ID number, x position, y position, and theta angle in radians. The ID number should be a whole number integer, while the x,y, and theta positions can be floats. 
The play field is sized at -1.2 to 1.0 in the x and -1.4 to 2.35 in the y so the x and y positions must be within those dimensions. 
There are currently 50 robots active in the Coachbot swarm so please limit your number of robots to 25.
The ID number must be between 0 and 99.
Each robot must start 25 cm away from each other.
The x and y positions are in meters while the theta value is in radians

Example init_poses.csv file:
0,0.75,1.0,3.14
1,0.5,0.5,1
2,-0.5,-0.5,0
3,-0.1,-0.1,1
4,-1,-1,0
5,-0.5,0.5,1
6,0.5,-0.5,1
7,0.25,0.25,3.14
8,-0.25,0.8,2.5
9,0.75,-0.1,1.5

If your init_poses.csv file does not abide to the rules above, you will receive an email and can check the input_pose_errors.csv file for details on where it failed


User Code
The user code file must be named usr_code.py and be written in python. This same code will be uploaded to every robot that will run your code. Section III below explains the available robot functions and various functionality such as sending messages and logging. Instead of a main function, the robot code will look for a function called usr(robot) so be sure to add that to your code as shown below. It is also good practice to add a small delay in the while loop (See Section III.A.8 for function specifics).

usr_code.py format:
imports
def usr(robot):
	# write code here
	while condition:
		robot.delay() # defaults to 20 ms which is sufficient
		# more code here
return condition

The current time limit for code runtime is 10 minutes so if your code hits this limit, it will pause the code and return all the information it received until that point.


Email
The email.txt file should simply contain the email address all alerts should be sent to. Users will receive an email notification when their code begins to run and one when it has completed. The User may also instead receive an email letting them know that their initial positions were invalid or their code hit the time limit.
GitHub Interface to the Coachbot System
The github repo to submit your code is located at https://github.com/dornadulavaishnavi/CoachbotSwarm/tree/main. Your code should be uploaded to the Code_Queue folder in the main branch of the repo. To gain access to push code to this repo, please send us an email to be added as a collaborator. Once you have access, clone the repo to your local computer using https or ssh. To avoid merge conflicts, please remember to git pull before you try to upload your code. Copy your required files into a uniquely named folder and be sure to note this name, as this will be the name of the folder containing the output of your run.

