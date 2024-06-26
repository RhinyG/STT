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
import org.firstinspires.ftc.teamcode.robotParts.movement.Localization.Localization;

import java.util.List;

@Config
@Autonomous(name = "demoDrive")
public class demoDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrivetrain drive = new MecanumDrivetrain(this);
        Localization odom = new Localization(drive.FrontL, drive.BackL, drive.FrontR, true);
        PathFollower bezierFollower = new PathFollower(this);
        pidFollower p2p = new pidFollower(this);

        double[] position,  // = {x, y, theta, velocity}
                outputFollower = {0,0,0}; // = {driveVectorR, driveVectorTheta, rotatePower}
        PathBuilder path1 = new PathBuilder(new double[][] {{0, 0}, {100, 0}, {100, 100}, {0, 100}}).buildPath();
        PathBuilder path2 = new PathBuilder(new double[][]{path1.lastPoint(), {0, 50}, {-50, 50}}).buildPath();
        PathBuilder path3 = new PathBuilder(path2.reversePoints());


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hardwareMap, false);

        int driveState = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            position = odom.arcVelocity();
            switch (driveState) {
                case 0:
                    outputFollower = p2p.followPID(position, path1.firstPoint(),0);
                    if (p2p.distanceToEndPoint(position, path1.firstPoint()) < 1) driveState++; //TODO: And velocity smaller than certain value?
                    break;
                case 1:
                    outputFollower = bezierFollower.followPath(path1, position, false);
                    if (bezierFollower.distanceToEndPoint(position, path1.lastPoint()) < 1) driveState++;
                    break;
                case 2:
                    outputFollower = bezierFollower.followPath(path2, position,1, true, true, Math.toRadians(90),90);
//                    if (arm.isDone){driveState++;}
                    break;
                case 3:
                    outputFollower = p2p.followPID(position,new double[]{-100, 50}, 90);
                    if (p2p.distanceToEndPoint(position, new double[]{-100, 50}) < 1) driveState++;
                    break;
                case 4:
                    outputFollower = bezierFollower.followPath(path3, position, true);
                    if (bezierFollower.distanceToEndPoint(position, path3.lastPoint()) < 1) driveState++;
            }
            drive.drive(new double[] {outputFollower[0],outputFollower[1]}, outputFollower[2]);
            telemetry.update();
        }
    }
}