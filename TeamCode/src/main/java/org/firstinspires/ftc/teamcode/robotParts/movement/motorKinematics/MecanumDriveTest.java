package org.firstinspires.ftc.teamcode.robotParts.movement.motorKinematics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
//TODO: delete if MecanumDriveTuner.java works
@Config
@TeleOp(name = "Mecanum drive Test", group = "Tests")
public class MecanumDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double[] driveVector;
        MecanumDrivetrain drive = new MecanumDrivetrain(this);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hardwareMap,false);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driveVector = drive.toPolar(gamepad1.left_stick_x,-gamepad1.left_stick_y);
            telemetry.addData("drive r ",driveVector[0]);
            telemetry.addData("drive theta ",driveVector[1]);
            drive.drive(driveVector,gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}