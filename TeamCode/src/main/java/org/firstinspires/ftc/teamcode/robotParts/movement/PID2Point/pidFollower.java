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

    /**
     * TODO: documentation
     * @param localization
     * @param targetPosition
     * @param targetHeading
     * @return An array with a maxSpeed, a TODO: robotCentric or fieldCentric? driveAngle and a rotation PD-controlled power.
     */
    public double[] followPID(double[] localization, double[] targetPosition, double targetHeading) {
        drive.setPID(driveP,driveI,driveD);//TODO: you might be able to delete this step, according to https://docs.ftclib.org/ftclib/features/controllers
        rotate.setPID(rotateP,rotateI,rotateD);

        drivePower = drive.calculate(distanceToEndPoint(localization, targetPosition), 0);
        rotatePower = rotate.calculate(localization[2],targetHeading); //localization[2] used to be targetPosition[2]

        driveAngle = Math.atan2(targetPosition[1] - localization[1],targetPosition[0] - localization[0]) - localization[2];

        telemetry();
        return new double[] {drivePower, driveAngle, rotatePower};
    }
    //TODO: documentation
    public void telemetry(){
        telemetry.addData("CurrentHeading", position[2]);
    }

    public void runOpMode() {}
}