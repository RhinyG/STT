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
import org.firstinspires.ftc.teamcode.robotParts.movement.motorKinematics.MecanumDrivetrain;

import java.util.List;

@Config
@TeleOp(name = "PathFollower Constants Tuner", group = "Tests")
public class PathFollowerConstantsTuner extends LinearOpMode {
    double[] values, savedValues, predictedStoppingVector;
    double time,speed, deltaDistance, predictedStoppingScalar, predictedStoppingVectorMagnitude;
    public static double decelerationConstant = 0.0008;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrivetrain drive = new MecanumDrivetrain(this);

        Localization odom = new Localization(drive.FrontL,drive.BackL,drive.FrontR,true);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        int state = 0;

        telemetry.addLine("Replace the keyword 'final' from PathFollower.java:19 with 'public static'");
        telemetry.addLine("This file is for tuning the PathFollower Constants.");
        telemetry.addLine("Step 1: press X/□");
        telemetry.addLine("Step 2: press Y/△");
        telemetry.addLine("Step 3: press B/○");
        telemetry.addLine("Step 4: press A/X");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.x) {
                odom.leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                state = 0;
                time = System.currentTimeMillis();
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
                    //TODO: addLines for explanation
                    //Full speed then coast.
                    values = odom.arcVelocity();
                    telemetry.addData("X", values[0]);
                    telemetry.addData("Y", values[1]);
                    telemetry.addData("Heading", values[2]);

                    if (time + 2000 < System.currentTimeMillis()) drive.drive(new double[] {1,0},0);
                    else {
                        time = System.currentTimeMillis();
                        drive.drive(new double[] {0,0},0);
                        state++;
                        savedValues = values;
                        break;
                    }
                case 1:
                    //TODO: addLines for explanation
                    //Calculate time necessary to slow down.
                    values = odom.arcVelocity();
                    if (Math.hypot(values[3], values[4]) < 1) {
                        state++;
                        time = System.currentTimeMillis() - time;
                        break;
                    }
                case 2:
                    //TODO: addLines for explanation
                    //Calculate distance travelled and decelerationConstant.
                    speed = Math.hypot(values[3],values[4]);
                    deltaDistance = Math.hypot(values[0] - savedValues[0],values[1] - savedValues[1]);

                    telemetry.addData("X travelled ", values[0] - savedValues[0]);
                    telemetry.addData("Y travelled ", values[1] - savedValues[1]);
                    telemetry.addData("Distance travelled ", deltaDistance);
                    telemetry.addData("Time ",time);
                    telemetry.addData("Starting velocity ",speed);
                    telemetry.addData("Deceleration Constant via dT ", -speed/time);
                    telemetry.addData("Deceleration Constant via dX ", -Math.pow(speed,2)/(2*deltaDistance));
                    telemetry.addData("Predicted stopping distance via kinematics formula", -Math.pow(speed,2)/(2*decelerationConstant));

                    predictedStoppingScalar = Math.hypot(values[3],values[4])/decelerationConstant;
                    predictedStoppingVector = new double[]{values[3] * predictedStoppingScalar, values[4] * predictedStoppingScalar};
                    predictedStoppingVectorMagnitude = Math.hypot(predictedStoppingVector[0], predictedStoppingVector[1]);

                    telemetry.addData("Wolfpack stopping distance formula ", predictedStoppingVectorMagnitude);
                    telemetry.addLine("Press a button to go to another step.");
                case 3:
                    //Tune weights for centrifugal, translational and drive force.
            }
            telemetry.update();
        }
    }
}