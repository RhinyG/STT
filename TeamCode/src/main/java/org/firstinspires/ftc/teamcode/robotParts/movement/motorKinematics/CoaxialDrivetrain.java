package org.firstinspires.ftc.teamcode.robotParts.movement.motorKinematics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

@Config
public class CoaxialDrivetrain extends RobotPart {
    double x, y, r, gamepadTheta, servoGoal, turn;
    public Servo servoFrontB,servoFrontR,servoBackB,servoBackR;
    public DcMotorEx FrontL,FrontR,BackL,BackR;
    public DcMotorEx[] DTMotors;
    String[] DTMotorNames = {"left_front","right_front","left_back","right_back"};
    public Servo[] DTServos;
    String[] DTServoNames = {"front_left","front_right","back_left","back_right"};
    Telemetry telemetry;
    LinearOpMode myOpMode;
    public CoaxialDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetry = opmode.telemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * This methods initialises the swerve drivetrain and the IMU and sets all the directions and modes to their correct settings.
     */
    //TODO: parameter boolean TeleOp
    public void initRobot(HardwareMap map) {
        DTMotors = new DcMotorEx[]{FrontL, FrontR, BackL, BackR};
        for (int i = 0; i < DTMotors.length; i++) {
            DTMotors[i] = map.get(DcMotorEx.class,DTMotorNames[i]);
            if (i < 2){
                DTMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                DTMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            DTMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DTMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        DTServos = new Servo[]{servoFrontB,servoFrontR,servoBackB,servoBackR};
        for (int i = 0; i < DTServos.length; i++) {
            DTServos[i] = map.get(Servo.class,DTServoNames[i]);
        }
        initIMU(map);
        DTMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void resetYaw() {
        imu.resetYaw();
    }
    public void singlePodSimple(){
        DTMotors[1].setPower(myOpMode.gamepad1.left_stick_y);
        DTServos[1].setPosition(myOpMode.gamepad1.left_trigger);
    }
    public void fullDrivetrainSimple(){
        x = myOpMode.gamepad1.left_stick_x;
        y = -myOpMode.gamepad1.left_stick_y;
        r = Math.sqrt(x * x + y * y);
        turn = myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger;

        if (x >= 0 && y >= 0) {
            gamepadTheta = Math.atan(y / x);
        } else if (x<0) {
            gamepadTheta = Math.atan(y / x) + Math.PI;
        } else {
            gamepadTheta = Math.atan(y / x) + 2 * Math.PI;
        }

        if(r > 0.2){
            servoGoal = gamepadTheta/(2*Math.PI);
        }

        if (myOpMode.gamepad1.x) {
            for (DcMotorEx motor : DTMotors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

//        if (gamepadTheta < 0.25 * Math.PI) {
//            DTMotors[0].setPower(r + turn);
//            DTMotors[1].setPower(r + turn);
//            DTMotors[2].setPower(r - turn);
//            DTMotors[3].setPower(r - turn);
//        } else if (gamepadTheta < 0.75 * Math.PI) {
//            DTMotors[0].setPower(r + turn);
//            DTMotors[1].setPower(r - turn);
//            DTMotors[2].setPower(r + turn);
//            DTMotors[3].setPower(r - turn);
//        } else if (gamepadTheta < 1.25 * Math.PI) {
//            DTMotors[0].setPower(r - turn);
//            DTMotors[1].setPower(r - turn);
//            DTMotors[2].setPower(r + turn);
//            DTMotors[3].setPower(r + turn);
//        } else if (gamepadTheta < 1.75 * Math.PI) {
//            DTMotors[0].setPower(r - turn);
//            DTMotors[1].setPower(r + turn);
//            DTMotors[2].setPower(r - turn);
//            DTMotors[3].setPower(r + turn);
//        } else {
//            DTMotors[0].setPower(r + turn);
//            DTMotors[1].setPower(r + turn);
//            DTMotors[2].setPower(r - turn);
//            DTMotors[3].setPower(r - turn);
//        }

        for (DcMotorEx motor : DTMotors) {
            motor.setPower(r);
        }
        for (Servo servo : DTServos) {
            servo.setPosition(servoGoal);
        }
        telemetry.addData("servoGoal",servoGoal);
        telemetry.addData("gamepadTheta",gamepadTheta);
        telemetry.addData("turn",turn);
        telemetry.addData("r",r);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
    }
    public void runOpMode() throws InterruptedException {}
}