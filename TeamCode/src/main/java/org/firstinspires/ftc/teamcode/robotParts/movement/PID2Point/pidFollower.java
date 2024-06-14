package org.firstinspires.ftc.teamcode.robotParts.movement.PID2Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

public class pidFollower extends RobotPart {
    Telemetry telemetry;
    double drivePower,driveAngle,rotatePower;
    public static double driveP = 0.0025, driveI = 0.001, driveD = 0.00004, rotateP = 0.0025, rotateI = 0.001, rotateD = 0.00004;
    PIDController drive = new PIDController(driveP, driveI, driveD), rotate = new PIDController(rotateP,rotateI,rotateD);
    double[] position;

    /**
     * This is the constructor.
     * @param opmode is opmode from a LinearOpMode file
     */
    public pidFollower(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    //TODO: documentation
    public double[] followPID(double[] startPosition, double[] endPosition, double targetHeading) {
        drive.setPID(driveP,driveI,driveD);//TODO: you might be able to delete this step, according to https://docs.ftclib.org/ftclib/features/controllers
        rotate.setPID(rotateP,rotateI,rotateD);

        drivePower = drive.calculate(distanceToEndPoint(startPosition, endPosition), 0);
        rotatePower = rotate.calculate(endPosition[2],targetHeading);

        driveAngle = Math.atan2(endPosition[1],endPosition[0]);

        telemetry();
        return new double[] {drivePower, driveAngle, rotatePower};
    }
    //TODO: documentation
    public void telemetry(){
        telemetry.addData("CurrentHeading", position[2]);
    }

    public void runOpMode() {}
}