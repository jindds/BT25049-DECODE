package org.firstinspires.ftc.teamcode.auto;

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
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Autonomous(name="BT-25049-RedAutonomous", group="comp")
public class Red9ballAuto extends LinearOpMode {

    public static Transfer.Configuration Params = new Transfer.Configuration();
    public static Shooter.PIDFConfiguration PIDFParams = new Shooter.PIDFConfiguration();

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
            Pose2d scorePose = new Pose2d(-25, 22, Math.toRadians(135));
            Pose2d firstLinePose = new Pose2d(-9, 21, Math.toRadians(90));
            Pose2d secondLinePose = new Pose2d(15, 24, Math.toRadians(90));
            Pose2d endPose = new Pose2d(0, 50, Math.toRadians(270));

            MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

            Action trajectoryAction = drive.actionBuilder(startPose)
                    .stopAndAdd(flywheel.fly())
                    .strafeToLinearHeading(scorePose.position, scorePose.heading)
                    .stopAndAdd(intake.spin())
                    // ball 1
                    .stopAndAdd(arm.extend())
                    .afterTime(0.1, arm.retract())
                    // ball 2
                    .afterTime(0.6, arm.extend())
                    .afterTime(0.7, arm.retract())
                    // ball 3
                    .afterTime(1.2, arm.extend())
                    .afterTime(1.3, arm.retract())
                    .waitSeconds(1.5)
                    .stopAndAdd(flywheel.stop())
                    // SPIKE I
                    .splineToLinearHeading(new Pose2d(firstLinePose.position, firstLinePose.heading), firstLinePose.heading)
                    .lineToY(55)
                    .stopAndAdd(flywheel.fly())
                    .afterTime(0.5, intake.pause())
                    .splineToLinearHeading(new Pose2d(scorePose.position, scorePose.heading), scorePose.heading)
                    .stopAndAdd(intake.spin())
                    // ball 1
                    .stopAndAdd(arm.extend())
                    .afterTime(0.1, arm.retract())
                    // ball 2
                    .afterTime(0.6, arm.extend())
                    .afterTime(0.7, arm.retract())
                    // ball 3
                    .afterTime(1.2, arm.extend())
                    .afterTime(1.3, arm.retract())
                    .waitSeconds(1.5)
                    .stopAndAdd(flywheel.stop())
                    // SPIKE II
                    .splineToLinearHeading(new Pose2d(secondLinePose.position, secondLinePose.heading), secondLinePose.heading)
                    .lineToY(70)
                    .lineToY(40)
                    .stopAndAdd(intake.pause())
                    .stopAndAdd(flywheel.fly())
                    .splineToLinearHeading(new Pose2d(scorePose.position, scorePose.heading), scorePose.heading)
                    .stopAndAdd(intake.spin())
                    // ball 1
                    .stopAndAdd(arm.extend())
                    .afterTime(0.1, arm.retract())
                    // ball 2
                    .afterTime(0.6, arm.extend())
                    .afterTime(0.7, arm.retract())
                    // ball 3
                    .afterTime(1.2, arm.extend())
                    .afterTime(1.3, arm.retract())
                    .waitSeconds(1.5)
                    .stopAndAdd(flywheel.stop())
                    .stopAndAdd(intake.pause())
                    .strafeToLinearHeading(endPose.position, endPose.heading)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            Actions.runBlocking(trajectoryAction);

        }

    }