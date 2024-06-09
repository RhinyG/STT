package org.firstinspires.ftc.teamcode.robotParts.movement.PID2Point;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class pidFollower {
    Telemetry telemetry;
    double driveDistance,targetHeading,drivePower,driveAngle,rotatePower;
    public static double driveP = 0.0025, driveI = 0.001, driveD = 0.00004, rotateP = 0.0025, rotateI = 0.001, rotateD = 0.00004;
    PIDController drive = new PIDController(driveP, driveI, driveD), rotate = new PIDController(rotateP,rotateI,rotateD);
    double[] position;

    /* *
     * This is the constructor.
     * @param opmode is opmode from a LinearOpMode file
     */
    public pidFollower(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
        //TODO: multipleTelemetry
    }
    //TODO: documentation
    public double[] followPID(double x, double y, double heading) {
        return followPID(new double[]{x, y, heading});
    }
    //TODO: documentation
    public double[] followPID(double[] position) {
        position[2] = targetHeading;

        drive.setPID(driveP,driveI,driveD);
        rotate.setPID(rotateP,rotateI,rotateD);

        driveDistance = Math.sqrt(Math.pow(position[0],2) + Math.pow(position[1],2));

        drivePower = drive.calculate(driveDistance);
        rotatePower = rotate.calculate(position[2],targetHeading);

        driveAngle = Math.atan2(position[1],position[0]);

        telemetry();
        return new double[] {drivePower, driveAngle, rotatePower};
    }
    //TODO: documentation
    public void telemetry(){
        telemetry.addData("CurrentHeading", position[2]);
    }
}