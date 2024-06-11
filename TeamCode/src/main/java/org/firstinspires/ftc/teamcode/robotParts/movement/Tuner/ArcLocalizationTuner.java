package org.firstinspires.ftc.teamcode.robotParts.movement.Tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robotParts.movement.Localization.Localization;

import java.util.List;

@Config
@TeleOp(name = "Localization Test", group = "Tests")
public class ArcLocalizationTuner extends LinearOpMode {
    double[] values;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx FrontL,FrontR,BackL;

        FrontL = hardwareMap.get(DcMotorEx.class, "left_front");
        FrontR = hardwareMap.get(DcMotorEx.class, "right_front");
        BackL = hardwareMap.get(DcMotorEx.class, "left_back");

        Localization odom = new Localization(FrontL,BackL,FrontR,true);
        //TODO: we also do this within each added class (like odom), is that necessary?
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        int state = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("You can test your tuning right now");
            telemetry.addLine("Step 1: press X/□");
            telemetry.addLine("Step 2: press Y/△");
            telemetry.addLine("Step 3: press B/○");
            telemetry.addLine("Step 4: press A/X");
            if (gamepad1.x) {
                state = 1;
                odom.leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad1.y) {
                state = 2;
                odom.leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad1.b) {
                state = 3;
                odom.leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad1.a) {
                state = 4;
                odom.leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            switch (state) {
                case 0:
                    telemetry.addLine("Before you start, get a caliper and add values for Lx, Ly, Rx, Ry, Bx, By. Also add values for your gear ratio, ticks per rotation and (odometry) wheel radius.");
                    telemetry.addLine("These values should be known for the hardware you use. Also make sure the back odometer is a reasonable distance from your turn axle.");
                    telemetry.addLine("You will then start step one. You have to turn the left and right wheels forward, and the back wheel to the left.");
                    telemetry.addLine("All encoder positions should increase.");
                    values = odom.arcVelocity();
                    telemetry.addData("X", values[0]);
                    telemetry.addData("Y", values[1]);
                    telemetry.addData("Heading", values[2]);
                case 1:
                    telemetry.addLine("You have to turn the left and right wheels forward, and the back wheel to the left.");
                    telemetry.addLine("All encoder positions should increase.");
                    values = odom.getPositionsTune();
                    telemetry.addData("currentLPos", values[0]);
                    telemetry.addData("currentRPos", values[1]);
                    telemetry.addData("currentBPos", values[2]);
                    telemetry.addLine("If step 1 has been tuned correctly, you can now drag the robot forward and get a semi-reasonable X value.");
                case 2:
                    telemetry.addLine("If step 1 has been tuned correctly, you can now drag the robot forward and get a semi-reasonable X value.");
                    telemetry.addLine("The next step is to roll the robot forward, preferably more than several meters, and check with a tape measure how close the variable is to reality.");
                    telemetry.addLine("To tune this, you need to change the odoMultiplier variable. You really want to get this to less than 1% error.");
                    telemetry.addData("Forward", odom.arcVelocity()[0]);
                    telemetry.addLine("The next step is to get the real value for your track width.");
                case 3:
                    telemetry.addLine("The next step is to get the real value for your track width (Ly - Ry).");
                    telemetry.addLine("One method is to rotate the robot 10 or 20 times, and compare how much your theta is versus how much it should be. If the number is too small, your track width needs to decrease.");
                    telemetry.addData("Heading",odom.arcVelocity()[2]);
                    telemetry.addLine("The next step is to tune Bx.");
                case 4:
                    telemetry.addLine("The next step is to get the real value for your Bx. Rotate the robot a full rotation, which should mean your back encoder is in the same spot as it started.");
                    telemetry.addLine("If your Y value does not return to zero, Bx needs to be changed. If Y is too high, increase/decrease Bx.");
                    telemetry.addData("Y coordinate",odom.arcVelocity()[1]);
                    telemetry.addLine("You should be done tuning.");
            }
            telemetry.update();
        }
    }
}