package org.firstinspires.ftc.teamcode.robotParts.movement.Bezier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.RobotPart;
import org.firstinspires.ftc.teamcode.robotParts.movement.PID2Point.pidFollower;

public class PathFollower extends RobotPart {
    //TODO: variable documentation
    //TODO: not everything has to be a double
    //TODO: make final after tuning
    public static double Fcent_weight, rotateP, rotateD, translationalP, translationD, decelerationConstant = 0.0007;
    double[] coordinate, derivative, sec_derivative;
    double coordinateLength;
    double trans_dist, shortest_dist, relative_pos;
    int closestT;
    double d2;
    double Fcent, Ftrans, Fcorr;
    double corrPower, driveAngle, rotatePower;
    double theta1, theta2, theta4;
    double robot_velocity_magnitude;
    double predictedStoppingScalar, predictedStoppingVectorMagnitude;
    double[] predictedStoppingVector, predictedStopPosition;
    PDController translational = new PDController(translationalP,translationD),rotate = new PDController(rotateP, rotateD);
    LinearOpMode myOpMode;
    public PathFollower(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetry = opmode.telemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    pidFollower p2p = new pidFollower(myOpMode);

    /**
     * TODO: documentation, EN
     * @param path
     * @param localization
     * @param maxSpeed
     * @param endSpline
     * @param headingLocked
     * @param endHeading
     * @param startCheckCoasting
     * @return An array with a maxSpeed, a TODO: robotCentric or fieldCentric? driveAngle and a rotation PD-controlled power.
     */
    public double[] followPath(PathBuilder path, double[] localization, double maxSpeed, boolean endSpline, boolean headingLocked, double endHeading, int startCheckCoasting) {
        coordinateLength = path.coordinate.length;
        if (closestT >= startCheckCoasting) {
            robot_velocity_magnitude = Math.hypot(localization[3],localization[4]);

            predictedStoppingScalar = robot_velocity_magnitude/decelerationConstant;
            predictedStoppingVector = new double[]{localization[3] * predictedStoppingScalar, localization[4] * predictedStoppingScalar};
            predictedStoppingVectorMagnitude = Math.hypot(predictedStoppingVector[0], predictedStoppingVector[1]);
        } else predictedStoppingVectorMagnitude = 0;

        if (predictedStoppingVectorMagnitude > distanceToEndPoint(localization, path.lastPoint()) && endSpline) {
            predictedStopPosition = new double[]{predictedStoppingVector[0] + localization[0], predictedStoppingVector[1] + localization[1], localization[2]};

            //TODO: this might not work because predictedStopPosition keeps refreshing.
            return p2p.followPID(predictedStopPosition, path.lastPoint(),endHeading);
        } else {
            //Calculates the point on the path closest to the robot, using the cached table.
            for (int i = closestT; i < coordinateLength; i++) {
                coordinate = path.coordinate[i];
                trans_dist = Math.pow(coordinate[0] - localization[0], 2) + Math.pow(coordinate[1] - localization[1], 2);
                if (i != 0) {
                    if ((shortest_dist == 0) || (Math.abs(trans_dist) < shortest_dist) && !Double.isNaN(trans_dist)) {
                        shortest_dist = Math.abs(trans_dist);
                        closestT = i;
                    }
                } else {
                    shortest_dist = Math.abs(trans_dist);
                    closestT = i;
                }
            }

            coordinate = path.coordinate[closestT];
            derivative = path.derivative[closestT];
            sec_derivative = path.sec_derivative[closestT];

            d2 = (sec_derivative[1] * derivative[0] - sec_derivative[0] * derivative[1]) / Math.pow(sec_derivative[0], 2);
            relative_pos = (coordinate[0] - localization[0]) + (coordinate[1] - localization[1]);

            Fcent = Fcent_weight * Math.pow(robot_velocity_magnitude, 2) / path.r_circle[closestT];
            Ftrans = translational.calculate(Math.sqrt(shortest_dist));

            if (d2 < 0) Fcent *= -1;
            if ((d2 < 0 && relative_pos < 0) || (d2 > 0 && relative_pos > 0)) Ftrans *= -1;

            Fcorr = Ftrans + Fcent;
            if (Fcorr > 1) Fcorr = 1;
            else if (Fcorr < 1) Fcorr = -1;
            else corrPower = Fcorr;

            //TODO: better names? Another option is deletion, you only use them once.
            theta1 = Math.asin(Fcorr / maxSpeed);
            theta2 = localization[2];
            theta4 = Math.atan(derivative[1] / derivative[0]);

            driveAngle = 0.5*Math.PI - theta1 - theta2 + theta4;
            if (headingLocked) rotatePower = rotate.calculate(localization[2],endHeading);
            else rotatePower = rotate.calculate(localization[2],driveAngle); //TODO: tune PD (can probably copy from P2P).
            return new double[] {maxSpeed,driveAngle,rotatePower};
        }
    }

    /**
     * TODO: documentation
     * @param path
     * @param localization
     * @param endSpline
     * @return An array with a maxSpeed, a TODO: robotCentric or fieldCentric? driveAngle and a rotation PD-controlled power.
     */
    public double[] followPath(PathBuilder path, double[] localization, boolean endSpline) {
        return followPath(path, localization,1,endSpline, false, 0, 90);
    }

    /**
     * You have to have this function to extend RobotPart because RobotPart extends LinearOpMode. It doesn't do anything, I believe.
     */
    public void runOpMode() {}
}