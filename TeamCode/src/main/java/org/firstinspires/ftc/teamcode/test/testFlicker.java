package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.CompTeleop;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="flickerTest", group="test")
public class testFlicker extends LinearOpMode {

    public static CompTeleop.Configuration Params = new CompTeleop.Configuration();
    public static CompTeleop.PIDFConfiguration PIDFParams = new CompTeleop.PIDFConfiguration();

    public static class intake {
        private DcMotorEx intake;

        public intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class Spin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(Params.INTAKE_SPEED);
                return false;
            }

        }

        public Action spin() {
            return new Spin();
        }

        public class Pause implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0.0);
                return false;
            }
        }

        public Action pause() {
            return new Pause();
        }
        public class Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-Params.INTAKE_SPEED);
                return false;
            }
        }
        public Action out() {
            return new Out();
        }
    }

    public static class flywheel {
        private DcMotorEx flywheel;

        public flywheel(HardwareMap hardwareMap) {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheel.setDirection(DcMotorEx.Direction.REVERSE);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            flywheel.setVelocityPIDFCoefficients(PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);
        }

        public class Fly implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double targetTickPerSec =
                        (PIDFParams.TARGET_RPM * PIDFParams.TICKS_PER_REV) / 60.0;

                flywheel.setVelocity(targetTickPerSec);
                return false;
            }
        }

        public Action fly() {
            return new Fly();
        }

        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel.setPower(0.0);
                return false;
            }
        }
        public Action stop() {
            return new Stop();
        }
    }
    public static class arm {
        private Servo arm;
        public arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(Servo.class, "arm");
            arm.setPosition(Params.startPosArm);
        }
        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                arm.setPosition(Params.ARM_RANGE);
                return false;
            }
        }

        public Action extend() {
            return new Extend();
        }
        public class Retract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPosition(Params.startPosArm);
                return false;
            }
        }
        public Action retract() {
            return new Retract();
        }

    }
    @Override
    public void runOpMode() throws InterruptedException {
        flywheel flywheel = new flywheel(hardwareMap);
        intake intake = new intake(hardwareMap);
        arm arm = new arm(hardwareMap);

        Pose2d startPose = new Pose2d(-51, 48, Math.toRadians(135));
        Pose2d scorePose = new Pose2d(-25, 20, Math.toRadians(135));
        Pose2d firstLinePose = new Pose2d(-12, 22, Math.toRadians(90));
        Pose2d secondLinePose = new Pose2d(12, 22, Math.toRadians(90));
        Pose2d endPose = new Pose2d(0, 50, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action trajectoryAction = drive.actionBuilder(startPose)
                .stopAndAdd(arm.extend())
                .afterTime(0.1, arm.retract())
                .afterTime(0.2, arm.extend())
                .afterTime(0.3, arm.retract())
                .build();

        waitForStart();

        Actions.runBlocking(trajectoryAction);

    }

}