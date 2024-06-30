package org.firstinspires.ftc.teamcode.robotParts.movement.Tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robotParts.movement.Localization.Localization;
import org.firstinspires.ftc.teamcode.robotParts.movement.PID2Point.pidFollower;
import org.firstinspires.ftc.teamcode.robotParts.movement.motorKinematics.MecanumDrivetrain;

import java.util.List;

@Config
@TeleOp(name = "P2P Tuner", group = "Tests")
public class PIDtoPointTuner extends LinearOpMode {
    double[] values;
    public static int state = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrivetrain drive = new MecanumDrivetrain(this);

        Localization odom = new Localization(drive.FrontL,drive.BackL,drive.FrontR,true);

        pidFollower pid = new pidFollower(this);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addLine("Replace the keyword 'final' from PathFollower.java:19 with 'public static'");
        telemetry.addLine("Step 1: press X/□");
        telemetry.addLine("Step 2: press Y/△");
        telemetry.addLine("Step 3: press B/○");
        telemetry.addLine("Step 4: press A/X");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            values = odom.arcVelocity();
            if (gamepad1.x) {
                state = 0;
                odom.leftOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad1.y) {
                state = 2;
                odom.leftOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad1.b) {
                state = 3;
                odom.leftOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad1.a) {
                state = 4;
                odom.leftOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.rightOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                odom.backOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            switch (state) {
                case 0:
                    if (pid.distanceToEndPoint(values, new double[] {0, 100}) > 0.1) drive.drive(pid.followPID(values, new double[]{0, 0}, 0));
                    else state++; break;
                case 1:
                    if (pid.distanceToEndPoint(values, new double[] {0, 0}) > 0.1) drive.drive(pid.followPID(values, new double[]{0, 0}, 0));
                    else state = 0; break;
                case 2:

                case 3:

                case 4:

            }
            telemetry.update();
        }
    }
}
