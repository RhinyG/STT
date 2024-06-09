package org.firstinspires.ftc.teamcode.robotParts.movement.Tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robotParts.movement.Localization.localization;

import java.util.List;

@Config
@TeleOp(name = "Localization Test", group = "Tests")
public class LocalizationTuner extends LinearOpMode {
    double[] position;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx FrontL,FrontR,BackL;

        FrontL = hardwareMap.get(DcMotorEx.class, "left_front");
        FrontR = hardwareMap.get(DcMotorEx.class, "right_front");
        BackL = hardwareMap.get(DcMotorEx.class, "left_back");

        localization odom = new localization(FrontL,BackL,FrontR,true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            position = odom.arcVelocity();
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("heading",Math.toDegrees(position[2]));
            telemetry.addData("currentLPos", odom.currentLPos);
            telemetry.addData("currentRPos", odom.currentRPos);
            telemetry.addData("currentBPos", odom.currentBPos);
            telemetry.update();
        }
    }
}