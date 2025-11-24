package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="BT-25049-Teleop", group="comp")
public class CompTeleop extends OpMode {
    public static class Configuration {
        public double MAX_DRIVE_SPEED = 1.0;
        public double ARM_RANGE = 0.6;
        public double INTAKE_SPEED = 0.85;
        public double HOOD_SPEED = 100.0;
        public double FLYWHEEL_SPEED = 0.8;
        public double startPosArm = 1.0;
    }
    public static CompTeleop.Configuration Params = new CompTeleop.Configuration();

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx intake, flywheel;
    public Servo arm, hood1, hood2;
    public IMU imu;
    private boolean leftBumperState, rightBumperState, intakeRunningForward, intakeRunningReverse, bButtonState, flywheelRunning;

    @Override
    public void init() {
        // motor
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servo
        arm = hardwareMap.get(Servo.class, "arm");
        // hood1 = hardwareMap.get(Servo.class, "hood1");
        hood2 = hardwareMap.get(Servo.class, "hood2");

        intakeRunningForward = false;
        intakeRunningReverse = false;
        flywheelRunning = false;
        arm.setPosition(Params.startPosArm);
    }

    @Override
    public void loop() {
        // intake and outake
        if (gamepad1.left_bumper && !leftBumperState) {
            if (intakeRunningForward) {
                intakeRunningForward = false;
            } else {
                intakeRunningForward = true;
                intakeRunningReverse = false;
            }
        }

        if (gamepad1.right_bumper && !rightBumperState) {
            if (intakeRunningReverse) {
                intakeRunningReverse = false;
            } else {
                intakeRunningReverse = true;
                intakeRunningForward = false;
            }
        }

        leftBumperState = gamepad1.left_bumper;
        rightBumperState = gamepad1.right_bumper;

        if (intakeRunningForward) {
            intake.setPower(Params.INTAKE_SPEED);
        } else if (intakeRunningReverse) {
            intake.setPower(-Params.INTAKE_SPEED);
        } else {
            intake.setPower(0.0);
        }

        // flywheel
        if (gamepad1.b && !bButtonState) {
            flywheelRunning = !flywheelRunning;
        }

        bButtonState = gamepad1.b;

        if (flywheelRunning) {
            flywheel.setPower(Params.FLYWHEEL_SPEED);
        } else {
            flywheel.setPower(0.0);
        }

        // arm servo mode
        if (gamepad1.a) {
            arm.setPosition(Params.ARM_RANGE);
        } else {
            arm.setPosition(Params.startPosArm);
        }
        // hood extend and retract servo mode
        if (gamepad1.x) {
            // hood1.setPosition(-(Params.HOOD_SPEED / 180.0));
            hood2.setPosition((Params.HOOD_SPEED / 180.0));
        }
        if (gamepad1.y) {
            // hood1.setPosition(Params.HOOD_SPEED / 180.0);
            hood2.setPosition(-(Params.HOOD_SPEED / 180.0));
        }
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void drive(double y, double x, double rx) {
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;
        double backLeftPower = y - x + rx;

        double maxPower = 1.0;

        maxPower = (Math.max(maxPower, Math.abs(frontLeftPower)));
        maxPower = (Math.max(maxPower, Math.abs(frontRightPower)));
        maxPower = (Math.max(maxPower, Math.abs(backRightPower)));
        maxPower = (Math.max(maxPower, Math.abs(backLeftPower)));

        frontLeft.setPower(Params.MAX_DRIVE_SPEED * (frontLeftPower / maxPower));
        frontRight.setPower(Params.MAX_DRIVE_SPEED * (frontRightPower / maxPower));
        backRight.setPower(Params.MAX_DRIVE_SPEED * (backRightPower / maxPower));
        backLeft.setPower(Params.MAX_DRIVE_SPEED * (backLeftPower / maxPower));
    }
}