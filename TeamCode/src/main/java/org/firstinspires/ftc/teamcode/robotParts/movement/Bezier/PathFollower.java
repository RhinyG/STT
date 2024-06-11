package org.firstinspires.ftc.teamcode.robotParts.movement.Bezier;

import com.arcrobotics.ftclib.controller.PDController;

import java.nio.file.Path;

public class PathFollower {
    final double decelerationConstant = 0.0007;
    final double max_speed = 1;
    double[] coordinate, derivative, sec_derivative;
    double coordinateLength;
    double trans_dist, shortest_dist, relative_pos;
    int closestT;
    double d2;
    double Fcent, Ftrans, Fcorr;
    double Fcent_weight, Ftrans_weight, rotateP, rotateD;
    double corrPower, pathPower, drivePower, driveAngle, rotatePower;
    double theta1, theta2, theta4;
    double[] robot_velocity = new double[2];
    double robot_velocity_magnitude;
    PDController rotate = new PDController(rotateP, rotateD);
    public double[] followPath(PathBuilder path, double[] localization, boolean endSpline) {
        return followPath(path, localization,endSpline, false, 0);
    }
    public double[] followPath(PathBuilder path, double[] localization, boolean endSpline, boolean headingLocked, double endHeading) {
        coordinateLength = path.coordinate.length;
        robot_velocity[0] = localization[3];
        robot_velocity[1] = localization[4];
        robot_velocity_magnitude = Math.hypot(robot_velocity[0],robot_velocity[1]);

        double targetDistance = Math.hypot(path.lastPoint()[0]-localization[0], path.lastPoint()[1]-localization[1]);

        double predictedStoppingScalar = robot_velocity_magnitude/decelerationConstant;
        double[] predictedStoppingVector = new double[]{robot_velocity[0] * predictedStoppingScalar, robot_velocity[1] * predictedStoppingScalar};
        double predictedStoppingVectorMagnitude = Math.hypot(predictedStoppingVector[0], predictedStoppingVector[1]);
        if (predictedStoppingVectorMagnitude >= targetDistance) {
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
            Ftrans = -Math.sqrt(shortest_dist) * Ftrans_weight;//TODO: pid?

            if (d2 < 0) Fcent *= -1;
            if ((d2 < 0 && relative_pos < 0) || (d2 > 0 && relative_pos > 0)) Ftrans *= -1;

            Fcorr = Ftrans + Fcent;
            if (Fcorr > 1) Fcorr = 1;
            else if (Fcorr < 1) Fcorr = -1;
            else corrPower = Fcorr;

            drivePower = max_speed;

            theta1 = Math.asin(Fcorr/*/motor.speed*/);
            //It should if the weights are tuned.
            theta2 = Math.toRadians(localization[2]);
            theta4 = Math.atan(derivative[1] / derivative[0]);

            driveAngle = 0.5*Math.PI - theta1 - theta2 + theta4;
            if (headingLocked) rotatePower = rotate.calculate(localization[2],endHeading);
            else rotatePower = rotate.calculate(localization[2],driveAngle); //TODO: tune PID
        } else {
            //TODO: do the coasting thingy
        }
        return new double[] {drivePower,driveAngle,rotatePower};
    }
    public double distanceToEndPoint() {
        return Math.sqrt(shortest_dist);
    }
}