# FRC2020Swerve
### 6/26/20:
1. Added a Bit of QOL Code
	- Made Lazy SparkMax Class
2. Made Rudimentary Trajectory Generator
	- Offboard Generator in Python
		- Parametric Quintic Hermite Splines
		- Calculates Waypoints with Timestamp and Location on Field
		- Calculates Arc Length and Total Estimated Time
	- Currently Saves Trajectory As a List to A .txt File
3. To Do
	- Add Motor Configs
	- Add Sensor Feedback
	- Add Code to Import Autonomous Trajectories
	- Add Code to Follow Autonomous Trajectories

### 6/17/20:
1. Fixed Bugs
	- Fixed a Build Error with Static Consts (Apparently it needs to be a constexpr)
	- Fixed "Efficient Module Turning" Code Logic Issue
2. Added Functionality to Swerve Module Class
	- Added angleOnTarget Functionality
3. To Do
	- Add Motor Configs
	- Add Sensor Feedback
	- Add Autonomous Functionality

### 6/15/20:
1. Made Basic Swerve Functionality
	- Made Basic Swerve Module Class
	- Made Basic Swerve Controller Class
	- Made Swerve Kinematics Calculator Class
	- Made PID for Swerve Module
	- Added a Majority of Teleop Functionality
2. Added Quality of Life Code (Improvements to CAN, PS4 Controller, Geometry Stuff, etc...)
	- Made Lazy TalonFX Class
	- Made Vector2d Class
	- Added PS4 & Universal Controller Class
	- Added DS Class
3. To Do
	- Need to Add Motor Configs
	- Need to Add Sensor Feedback
