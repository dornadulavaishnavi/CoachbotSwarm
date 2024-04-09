# Coachbot Automated Swarm Testbed: Quickstart

The Coachbot Swarm and this repo is owned and maintained by the Rubenstein Lab at Northwestern University. If you have any questions, comments or concerns, please email us at coachbotswarmsystem@gmail.com and someone from our team will get back to you. We ask that you respect the system and help us keep it accessible to fellow swarm enthusiasts by adhering to the guidelines explained in this guide. We reserve the right to refuse access to any individuals who are not courteous to the system or the Coachbot team.

Check the user guide for more detailed information about how to use the system and troubleshooting: ***https://dornadulavaishnavi.github.io/coachbotuserguide.github.io/***

## Required Installs
### Python
Our swarm runs algorithms written in the Python programming language (python 2.7.16). To install python, please visit the official downloads page (https://www.python.org/downloads/) and select the appropriate OS for your machine and scroll down to find the python 2.7.16 release. Once the python set up prompt is complete, ensure that everything is installed properly by opening command prompt (or OS equivalent) and running *python --version*. This should output python 2.7.16 in the terminal. 

If using a later version of python, please make sure that any libraries used in the algorithm are available in the python version specified above.

To begin coding, we recommend using an IDE (Integrated Development Environment). If the reader doesn't have a currently prefered IDE, we recommend Visual Studio Code (https://code.visualstudio.com/download) while is widely used and available for Windows, Mac, and Ubuntu. 

### Github
For our testbed, we currently use the popular system, Github, as a platform for users to submit their experiments to and receive results. To get started, make an account on https://github.com/ (a free account will be sufficient for our system). There are two popular ways to use Github on your local machine, which are explained below.

#### Github Desktop
Github Desktop is a convenient interface to utilize git tools without needing to use the terminal. Download the tool from https://desktop.github.com/. Look for the “Current Repository” Menu on the top left below the banner. In that dropdown menu, click “add” and select “Clone repository…”.  In the Github repo you wish to clone, select the green “< > Code” dropdown button and copy either the https or ssh link. Which to choose is a matter of personal preference, but explore this reference to learn more about the differences: https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories. Once you have either the https or ssh link copied, go to Github Desktop and paste this link in the “URL” section. Make sure your local path points to where you want the repo to be housed and click the blue “clone” button. Now you should be able to pull by clicking “Pull Origin” in the top banner, see your changes on the left bar, commit changes in the bottom left corner, and push code with the button in the top banner. The option to push will not appear if you have no changes. Please remember to always pull before you push and save a local copy of your code outside the repo.  

#### Terminal
To use git through your preferred terminal, you must first download the appropriate packages. Follow this link to download Git for your OS: https://github.com/git-guides/install-git. Once you have downloaded git and checked for successful installation, you can clone the repo. In the Github repo you wish to clone, select the green “< > Code” dropdown button and copy either the https or ssh link. Which to choose is a matter of personal preference, but here is a reference to learn more about the differences: https://docs.github.com/en/get-started/getting-started-with-git/about-remote-repositories.  In your terminal, navigate to where you want the local repo to be housed. Type *git clone* and paste the https or ssh link and hit enter. This will clone the repo to your local folder and you can now make changes to the files or folders as desired on oyur local machine. The *git status* command highlights any changes on your local system that have not been sent to the repo. To contribute your changes to the repository so that others may access it, first *git add* the files that are shown as changed when you ran *gt status*. Once those changes have been added, run *git commit -am ""* with a message between those quotes briefly explaining your changes. Finally now that your changes have been committed, you can use *git pull* to make sure there will be no overriding issues and then *git push* your changes to the repository. Please remember to always run the *git pull* command before you *git push* and save a local copy of your code outside the repo. A quick guide to important and commonly used git commands can be found here: https://training.github.com/downloads/github-git-cheat-sheet/. The commands that will most commonly be used are *git pull, git status, git add, git commit*, and *git push*.

## Obtaining Contributer Access to the Repo

To gain access to push your code to the repo, email us at coachbotswarmsystem@gmail.com with your name, university, and github username and/or email address associated with the account.

## Getting Familiar with the System through the Example Folder
Once you clone the Coachbot Swarm repository to your local machine, the first place to help get familiar with the test bed is the *Example_Folder*. Further sections below walk through the files needed for submission and the available robot API functions.

## Writing Code for the Coachbot Swarm
### Experiment Code (usr_code.py)
The experiment code must be written in a particular format to run properly on the swarm and every robot in the experiment will run the same user code. Start by copying the usr_code.py file from the Example Folder. This file contains the proper format to begin writing an algorithm for the robots. The robots run the *def usr(robot)* function similar to a *main* function. Within this function, users write a *while True* loop for the robots to continuously execute until some user specified return condition is met. We highly recommend writting statements into the *experiment_log* folder to aid in debugging.

#### Robot API Functions
These functions are available for a user to access the robot's capabilities and perform tasks such as motion, messaging, and localization. Check out our API Overview tutorial here for a quick introduction: [https://youtu.be/KC8QtUyUukE](https://youtu.be/Z8qkd0gtyGM)

##### robot.set_vel(left,right)

Parameters: left and right should be whole numbers between -50 and 50 that indicate wheel speeds. 0 being no movement and 50 being the fastest possible speed. The negative values indicate that the wheel should spin backwards at that speed. These values have a unit of 

Output: none

Example: robot.set_vel(30,-40)

##### robot.set_led(r,g,b)

Parameters: r,g,b should be integers between and 0 and 100 to set the color and brightness of the onboard LED

Output: none

Example: robot.set_led(30,100,0)

##### robot.id (simulation) or robot.virtual_id(physical system)

Parameters: none

Output: An integer that is the virtual ID of the robot. 

Example: is = robot.id

##### robot.get_clock()

Parameters: none

Output: a float of the number of seconds elapsed since the program started

Example: curr_time = robot.get_clock()

##### robot.send_msg(msg)

Parameters: msg should be a string that is less than 64 bytes or it will be truncated. This msg can be the output of the struck.pack() function explained in the section below.

Output: True is successful, False if not

Example: robot.send_msg(struct.pack(‘fffii’, float_0, float_1, float_2, int_0, int_1))

##### robot.recv_msg()

Parameters: none

Output: Returns the messages in the buffer since the last call of this function. 

Example: msgs = robot.recv_msg()

##### robot.get_pose()

Parameters: none

Output: a list with the [x,y,theta], check to see that this output is valid before using it

Example: pose = robot.get_pose()

##### robot.delay()

Parameters: default is 20ms but a different integer parameter can be specified

Output: none

Example: robot.delay(500)

### Initial Swarm Positions (init_pose.csv)
This file specifies the starting position of each robot needed in the experiment. The values specified must follow the rules listed below. The x and y positions are in meters while the theta value is in radians (-2π, 2π). The arena orientation and sizing is further specified in Section I.A.1 of the User Guide linked at the top of this quickstart.
- Each row should specify an ID number, x position, y position, and theta angle in radians. The ID number should be a whole number integer, while the x,y, and theta positions can be floats. 
- The play field is sized at -1.2m to 1.0m in the x and -1.4m to 2.35m in the y so the x and y positions must be within those dimensions. 
- There are currently 50 robots active in the Coachbot swarm so please limit your number of robots to 50.
- The ID number must be between 0 and 49.
- Each robot must start 25 cm away from each other.

If your init_pose.csv file does not abide by the rules above, you will receive an email and can check the input_pose_errors.csv file for details on where it failed. 

## Acessing and Running Experiments on the Coachbot Swarm Simulator
Our testbed features a corresponding simulator that is compatible with Windows, Mac, and Linux machines. This tool is publicly located through Github and can be used to test algorithms at faster speeds and larger swarm sizes before being run on the physical testbed (***https://github.com/michelleezhang/swarm_simulation/tree/master***). Exact instructions on using this simulator are located in the README of its repository. Once the results on the simulator match the expectations of the user, it is then ready to be run on the physical testbed.

## Changing the Code to Run on the Physical Coachbot Swarm System
To effectively change the code of the algorithm from running on the simulator to the physical testbed, a few function names currently need to be changed. The physical robots use the function robot.virtual_id() to access their ID numbers instead of the simulator's equivalent function, robot.id. Similarly, instead of print statements used in the simulator, the physical testbed can track these statements through the user writing these statements in a log.write() as shown in the example *usr_code.py*.

## Submitting Code to be Run on the Physical Coachbot Swarm System

Every submission to our system must be a folder with three files. The first is a .txt file called email.txt which simply contains the email address that should be contacted about the submitted code. The second file specifies the initial positions of the robots before the user code is run. Specifications of that file’s format and restrictions on robot positions are outlined above and in Section II.A.1 of the User Guide. The third mandatory file is the code that will be uploaded to all active robots. This file, usr_code.py, must be written in python and formatted in the way outlined above and in Section II.A.2 of the User Guide. After placing these 3 required files into a uniquely named folder, copy this folder into the Code_Queue folder of the repository and push it to the github repository to be run. See the Github section above or in the User Guide for a refresher on how to contribute code to a repository. Once your code has been pushed to the CoachbotSwarm repository, you have successfully submitted to the testbed! Now you can sit back and eagerly await your experiment results!

An example submission with these three mandatory files are located within the Example_Folder/Submission_Examples directory of the repository. These files will showcase various robot functionalities and are a good place to start familiarizing yourself with our platform and robot capabilities. To begin experimenting with the platform, we recommend taking a look at these files and editing them to create your first submission. Try changing the initial positions and number of robots in the init_pose.csv file or edit parameters to change behaviors such as how long the robots drive for or their LED color to understand how the API calls work. Be sure to change the email in the email.txt appropriately.

Check the User Guide for more details on the guidelines of these input files.

## What to Expect from Running an Experiment on the Physical Coachbot Swarm System

The results of your algorithm will be in the Completed_Runs folder of the repo. Once you pull the repo, you can navigate in to the Completed_Runs folder to your folder. If you forget the name of your folder, it will be sent in the email you received notifying you that the code was done running. Within this folder, you should see the following directory tree (new files and directories are in bold).
	
<p>OriginalFolderName <br>
&emsp; usr_code.py <br>  
&emsp; init_poses.csv <br>
&emsp; email.txt <br>
&emsp; <strong>init_pose_errors.csv</strong> <br>
&emsp; <strong>output_logs</strong> <br>
&emsp; &emsp; <strong>ID_mapping.csv</strong> <br>
&emsp; &emsp; <strong>camera_video.mp4</strong> <br>
&emsp; &emsp; <strong>#_logging.csv</strong> <br>
&emsp; &emsp; <strong>#</strong> <br>
&emsp; &emsp; <strong>automation_errors</strong></p>

The <strong>init_pose_errors.csv</strong> will contain the contents of <strong>init_poses.csv</strong> or list any issues with the initial poses specified. The output_logs folder will hold all the outputs from the algorithm run. Since the ID you specify in the <strong>init_poses.csv</strong> file might not match the physical robot ID, the <strong>ID_mapping.csv</strong> file specifies which robot corresponds to which virtual ID. The .mp4 file is the recording of the run from our overhead raspberry pi camera. The logging files will be named with the virtual ID of the robot it pertains to and contain the position of the corresponding robot at every timestep of the run. This csv file is formatted in a timestep, x position, y position, theta angle in radians for each line. The <strong>#</strong> file is the virtual ID of the pertaining robot and will have any information you choose to write to the <strong>experiment_log</strong> file in your code. The <strong>automation_errors</strong> file will list any high level errors such as runtime limits or robots trying to exit the play field.
