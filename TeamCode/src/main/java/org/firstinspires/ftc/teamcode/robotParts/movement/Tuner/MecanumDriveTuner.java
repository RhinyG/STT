package org.firstinspires.ftc.teamcode.robotParts.movement.Tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robotParts.movement.motorKinematics.MecanumDrivetrain;

import java.util.List;

@Config
@TeleOp(name = "Mecanum drive Test", group = "Tests")
public class MecanumDriveTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double[] driveVector;
        int state = 0;
        double time = 0;
        MecanumDrivetrain drive = new MecanumDrivetrain(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hardwareMap,true);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    telemetry.addLine("To start tuning, press X/□");
                    telemetry.addLine("This will turn on all motors to power 0.3. They should all drive forwards.");
                    telemetry.addLine("If they don't, reverse motors in init() until they do.");
                    if (gamepad1.x) {
                        state++;
                        for (DcMotorEx motor : new DcMotorEx[]{drive.FrontL, drive.FrontR, drive.BackL, drive.BackR}) {
                            motor.setPower(0.3);
                        }
                    }
                case 1:
                    telemetry.addLine("If all motors are going forwards, press Y/△");
                    telemetry.addLine("The robot will drive forwards at full speed for 5 seconds. Record this speed.");
                    if (gamepad1.y) {
                        state++;
                        for (DcMotorEx motor : new DcMotorEx[]{drive.FrontL, drive.FrontR, drive.BackL, drive.BackR}) {
                            motor.setPower(1.0);
                            time = System.currentTimeMillis();
                        }
                    }
                case 2:
                    if (System.currentTimeMillis() - time > 1000) {
                        state++;
                    }
                case 3:
                    telemetry.addLine("For the next step, press X/□");
                    telemetry.addLine("The robot will strafe at full speed for 5 seconds. Record this speed.");
                    if (gamepad1.y) {
                        state++;
                        for (DcMotorEx motor : new DcMotorEx[]{drive.FrontL, drive.FrontR, drive.BackL, drive.BackR}) {
                            motor.setPower(1.0);
                            time = System.currentTimeMillis();
                        }
                    }
                case 4:
                    if (System.currentTimeMillis() - time > 1000) {
                        state++;
                    }
                case 5:
                    telemetry.addLine("You should now have a working drivetrain, and two speeds. Fill in those speeds at MecanumDrivetrain.java:20, y first, then x.");
            }
            telemetry.update();
        }
    }
}
