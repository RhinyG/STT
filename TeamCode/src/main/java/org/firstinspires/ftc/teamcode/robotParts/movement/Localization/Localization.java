package org.firstinspires.ftc.teamcode.robotParts.movement.Localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Localization {
    public DcMotorEx leftOdo,rightOdo,backOdo;
    public Localization(DcMotorEx left, DcMotorEx right, DcMotorEx back, boolean reset) {
        leftOdo = left;
        rightOdo = right;
        backOdo = back;

        if (reset) {
            leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    final double[]
            L = {-2.6, 16.5},
            R = {-2.6,-8},
            B = {17, -2.5};
    final double
            WHEEL_RADIUS = 48,//odometry wheel radius in millimeters
            GEAR_RATIO = 1/13.7,//TODO: wait what this shouldn't have any effect on odometry?
            TICKS_PER_ROTATION = 8192,
            odoMultiplier = (72/38.6),
            ticksPerCM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS); //about 690 ticks per centimeter
    //Pos are in ticks, the rest are in centimeters or radians.
    double currentLPos,currentRPos,currentBPos,oldX,oldY,oldTheta,currentX,currentY,currentTheta,oldLPos,oldRPos,oldBPos,dL,dR,dB,relDX,relDY,rSTR,rFWD,dForward,dTheta,dStrafe,oldTime,currentTime;

    /**
     * <i>Based on <a href="https://www.youtube.com/watch?v=ixsxDn_ddLE&t=3s">The Clueless | Intro to Odometry</a>.</i>
     * <p>This is the Constant Velocity Arc Localization algorithm STT uses. It uses three odometry wheels and no IMU. There is some line by line documentation
     * inside the method.</p><p></p>
     * <h2>Tuning</h2>
     * <p>It is very important to spend enough time on your tuning. If the earlier steps are not accurate, the later steps also won't be.</p><p></p>
     * <h3>Step 0</h3>
     * <p>Get a caliper and add values for Lx, Ly, Rx, Ry, Bx, By. Also add values for your gear ratio, ticks per rotation and (odometry) wheel radius.
     * These values should be known for the hardware you use. Also make sure the back odometer is a reasonable distance from your turn axle.
     * Replace the keyword 'final' from line 19 and line 23 with 'public static'</p><p></p>
     * <h3><a href="#getPositionsTune()">Step 1</a></h3>
     * <p>Equal your odometers to the motors they're plugged into. Check if the odometer directions are good.
     * If you move the wheels manually, do they increase like they should, or do they decrease?
     * If so, you need to add or remove a negative before the Odo.getCurrentPosition.</p><p></p>
     * <h3>Step 2</h3>
     * <p>If step 1 has been tuned correctly, you can now drag the robot forward and get a semi-reasonable X value.
     * The next step is to roll the robot forward, preferably more than several meters, and check with a tape measure how close the variable is to reality.
     * To tune this, you need to change the odoMultiplier variable. You really want to get this to less than 1% error.</p><p></p>
     * TODO: if too large, make odoMultiplier bigger/smaller (also add this to README.md and LocalizationTuner.java).
     * <h3>Step 3</h3>
     * <p>The next step is to get the real value for your track width (Ly - Ry). One method is to rotate the robot 10 or 20 times,
     * and compare how much your theta is versus how much it should be. If the number is too small, your track width needs to decrease.</p><p></p>
     * <h3>Step 4</h3>
     * <p>The next step is to get the real value for your Bx. Rotate the robot a full rotation, which should mean your back encoder is in the same spot as it started.
     * If your Y value does not return to zero, Bx needs to be changed. If Y is too high, increase/decrease Bx.</p><p></p>
     * TODO: if too large increase/decrease Bx (also add this to README.md and LocalizationTuner.java).
     * <h3>Step 5</h3>
     * <p>You should be done tuning. Change the public static variables back to final.</p>
     * @return An array containing the global X [0] and Y [1] coordinates and heading [2] of the robot on the field.
     * Also contains the X [3] and Y [4] speed components.
     */
    public double[] arcVelocity() {
        //Moves the current values to the old ones.
        oldTheta = currentTheta;
        oldX = currentX;
        oldY = currentY;
        oldLPos = currentLPos;
        oldRPos = currentRPos;
        oldBPos = currentBPos;
        oldTime = currentTime;

        //Updates the current values (should be via a Bulk read).
        getPositions();
        currentTime = System.currentTimeMillis();

        //Translates a position in ticks into a delta in centimeters.
        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        //Robot centric variable that calculates the delta forward. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dForward = (dR * L[1] - dL * R[1])/(L[1] - R[1]);
        //Robot centric variable that calculates the delta turn.
        dTheta = (dR - dL)/(L[1] - R[1]);
        //Robot centric variable that calculates the delta strafe, correcting for the turn.
        dStrafe = dB - B[0] * dTheta;
        //If dTheta == 0, revert to linearLocalization because then you have a circle with infinite diameter, which it can't (because it divides by zero).
        // Luckily a circle with infinite diameter is just a line. Realistically this probably won't happen, but it's always good to catch the error.
        if (dTheta == 0) {
            relDX = dForward;
            relDY = dStrafe;
            //Else, use arcLocalization. That means doing some fancy circle math.
        } else {
            rFWD = dForward / dTheta;
            rSTR = dStrafe / dTheta;

            relDX = rFWD * Math.sin(dTheta) - rSTR * (1 - Math.cos(dTheta));
            relDY = rSTR * Math.sin(dTheta) + rFWD * (1 - Math.cos(dTheta));
        }

        //Add the delta's to your current position and return those.
        //The fourth term is the current velocity in meters/second. Since the distance is in centimeters and time in millimeters, a (100)/(1/1000) = 10 term is added.
        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);
        return new double[] {currentX, currentY, currentTheta, 10*relDX/(currentTime-oldTime), 10*relDY/(currentTime-oldTime)};
    }

    /**
     * <a href="https://www.youtube.com/watch?v=ixsxDn_ddLE&t=3s">The Clueless linear odometry</a> tracking equations.
     * For tuning and documentation, see {@link #arcVelocity()}
     */
    @Deprecated
    public double[] linearVelocity() {
        oldTheta = currentTheta;
        oldX = currentX;
        oldY = currentY;
        oldLPos = currentLPos;
        oldRPos = currentRPos;
        oldBPos = currentBPos;

        currentLPos = -leftOdo.getCurrentPosition();
        currentRPos = rightOdo.getCurrentPosition();
        currentBPos = backOdo.getCurrentPosition();

        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        dForward = (dR * L[1] - dL * R[1])/(L[1] - R[1]); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(L[1] - R[1]);
        dStrafe = dB - B[0] * dTheta;

        relDX = dForward;
        relDY = dStrafe;

        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);

        return new double[] {currentX, currentY, currentTheta};
    }

    /**
     * This method is used by ArcLocalizationTuner.java.
     * It is step one of the tuning process.
     * @see <a href="#arcVelocity()">arcVelocity()</a>
     */
    public double[] getPositionsTune() {
        getPositions();
        return new double[] {currentLPos,currentRPos,currentBPos};
    }

    /**
     * Updates the current values (should be via a Bulk read).
     */
    public void getPositions() {
        currentLPos = -leftOdo.getCurrentPosition();
        currentRPos = rightOdo.getCurrentPosition();
        currentBPos = backOdo.getCurrentPosition();
    }
}
