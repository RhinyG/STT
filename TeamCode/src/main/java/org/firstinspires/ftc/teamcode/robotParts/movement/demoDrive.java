package org.firstinspires.ftc.teamcode.robotParts.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robotParts.movement.motorKinematics.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.movement.Bezier.PathBuilder;
import org.firstinspires.ftc.teamcode.robotParts.movement.Bezier.PathFollower;
import org.firstinspires.ftc.teamcode.robotParts.movement.PID2Point.pidFollower;
import org.firstinspires.ftc.teamcode.robotParts.movement.Localization.localization;

import java.util.List;

@Config
@Autonomous(name = "demoDrive")
public class demoDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrivetrain drive = new MecanumDrivetrain(this);
        localization odom = new localization(drive.FrontL, drive.BackL, drive.FrontR, true);
        PathFollower bezierFollower = new PathFollower();
        pidFollower p2p = new pidFollower(this);
        arm arm = new arm();

        double[] position,  // = {x, y, theta, velocity}
                outputFollower = {0,0,0}; // = {driveVectorR, driveVectorTheta, rotatePower}
        PathBuilder path1 = new PathBuilder({{0, 0}, {100, 0}, {100, 100}, {0, 100}});
        PathBuilder path2 = new PathBuilder({path1.endPoint, {0, 50}, {-50, 50}});

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hardwareMap, false);
        arm.init(hardwareMap,false);

        int driveState = 0, armState = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            position = odom.arcVelocity();
            switch (driveState) {
                case 0:
                    outputFollower = p2p.followPID(path1.startPoint);
                    if (p2p.distance < 1) {driveState++;}
                case 1:
                    outputFollower = bezierFollower.followPath(path1, position, endSpline = false, headingLocked = false);
                    if (path1.distanceToEndPoint < 1) {driveState++;}
                    break;
                case 2:
                    outputFollower = bezierFollower.followPath(path2, position, endSpline = true, headingLocked = true, endHeading = Math.toRadians(90));
                    if (arm.isKlaar){driveState++;} //For example
                case 3:
                    outputFollower = p2p.followPID(-100, 50, 90);
                    if (p2p.distance < 1) {driveState++;}
                case 4:
                    //Do other shit, etc.
            }
            drive.drive(new double[] {outputFollower[0],outputFollower[1]}, outputFollower[2]);
            telemetry.update();
        }
    }
}