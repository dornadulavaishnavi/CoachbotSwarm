Full User Guide is located at: https://docs.google.com/document/d/1fQcU9cK_8bXtfPoXu4C7zOwC8WK4yg2npxJTBvi9Fgg/edit?usp=sharing


Getting the Results
When your code has finished running, you will receive an email from the system letting you know so. When you pull the repo, the Completed_Runs folder will have your results in your folder with the files explained below.
Result Format
Within this folder, you should see the following directory tree (new files and directories are underlined).
	OriginalName_completed
		usr_code.py
		init_poses.csv
		email.txt
		init_pose_errors.csv
		output_logs
			ID_mapping.csv
			camera_video.mp4
		#_logging.csv
		#
		automation_errors
The init_pose_errors.csv will contain the contents of init_poses.csv or list any issues with the initial poses specified. The output_logs folder will hold all the outputs from the algorithm run. Since the ID you specify in the init_poses.csv file might not match the physical robot ID, the ID_mapping.csv file specifies which robot corresponds to which virtual ID. The .mp4 file is the recording of the run from our overhead raspberry pi camera. The logging files will be named with the virtual ID of the robot it pertains to and contain the position of the corresponding robot at every timestep of the run. This csv file is formatted in a timestep, x position, y position, theta angle in radians for each line. The # file is the virtual ID of the pertaining robot and will have any information you choose to write to the experiment_log file in your code. The automation_errors file will list any high level errors such as runtime limits or robots trying to exit the play field.


Accessing the Output
Once you get an email that your code has finished running, you will be able to access your code’s output files in the Completed_Runs folder of the repo. Use the git pull command to load the folders onto your local machine and navigate to the folder with the unique name you had uploaded to the Code_Queue folder. Once you copy the contents of your folder to somewhere outside your repo, please delete your folder from the Completed_Runs and push your code. (Remember to always pull before pushing to avoid merge conflicts).

Automation Error Messages:
The automation_errors file will contain information of high level errors such as hitting the runtime limit or robots leaving the playfield. This will not have any errors your user code may hit so please be sure to use the logging features to assist with debugging. 

Runtime limit: If you receive an email saying that your code hit the time limit, please make sure your code hits its return conditions properly or reduce the runtime of your code. You will still receive all the information above for the length of time your algorithm ran and access it in the same way.

Robot out of Bounds: If you receive an email or see in your automation_errors file that some robots went out of bounds, please make sure that your code keeps the robots within the playfield. The message in the file should specify the offending robot and the position it left the field to help with debugging. You will still receive all the information above for the length of time your algorithm ran and access it in the same way.

