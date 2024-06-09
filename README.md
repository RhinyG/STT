## NOTICE
This repository is a copy of the FTC SDK for the CENTERSTAGE (2023-2024) competition season. You might want to read their [README](https://github.com/FIRST-Tech-Challenge/FtcRobotController)

## Downloading the Project
If you are an Android Studio programmer, there are several ways to download this repo. Note that if you use the Blocks or OnBot Java Tool to program your robot, then you do not need to download this repository.

* If you are a git user, you can clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.

## Tuning our motion algorithm
This project contains STT's Bézier Curve motion algorithm, a Guided Vector Field algorithm.

Firstly, you should tune your drivetrain and the config file on the Driver Hub.
* Mecanum: tune [MecanumDrivetrain](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robotParts/movement/motorKinematics/MecanumDrivetrain.java) by using [MecanumDriveTuner](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robotParts/movement/Tuner/MecanumDriveTuner.java).
* For differential swerve, tune [DifferentialDrivetrain](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robotParts/movement/motorKinematics/DifferentialDrivetrain.java) by using (as of now non-existent).
* For coaxial swerve, tune [CoaxialDrivetrain](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robotParts/movement/motorKinematics/CoaxialDrivetrain.java) by using (as of now non-existent).

Then, you should tune the [localization algorithm](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robotParts/movement/Localization/localization.java) by using [LocalizationTuner](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robotParts/movement/Tuner/LocalizationTuner.java).

It is very important to spend enough time on your tuning. If the earlier steps are not accurate, the later steps also won't be.

* Step 0

Get a caliper and add values for Lx, Ly, Rx, Ry, Bx, By. Also add values for your gear ratio, ticks per rotation and (odometry) wheel radius.

This values should be known for the hardware you use. Also make sure the back odometer is a reasonable distance from your turn axle.

* Step 1

Equal your odometers to the motors they're plugged into. Check if the odometer directions are good.

If you move the wheels manually, do they increase like they should, or do they decrease?

If so, you need to add or remove a negative before the Odo.getCurrentPosition.

* Step 2

If step 1 has been tuned correctly, you can now drag the robot forward and get a semi-reasonable X value. 

The next step is to roll the robot forward, preferably more than several meters, and check with a tape measure how close the variable is to reality.

To tune this, you need to change the odoMultiplier variable. You really want to get this to less than 1% error.

* Step 3

The next step is to get the real value for your track width (Ly - Ry). One method is to rotate the robot 10 or 20 times,

and compare how much your theta is versus how much it should be. If the number is too small, your track width needs to decrease.

* Step 4

The next step is to get the real value for your Bx. Rotate the robot a full rotation, which should mean your back encoder is in the same spot as it started.

If your Y value does not return to zero, Bx needs to be changed. If Y is too high, increase/decrease Bx.