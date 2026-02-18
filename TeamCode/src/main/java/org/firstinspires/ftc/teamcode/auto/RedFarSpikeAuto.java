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
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Autonomous(name="RedFarSpikeAuto", group=("comp"))
public class RedFarSpikeAuto extends LinearOpMode {

    public static Transfer.Configuration Params = new Transfer.Configuration();
    public static class PIDFConfiguration {
        public double kP = 200.0;
        public double kI = 0.0;
        public double kD = 0.0;;
        public double kF = 15.4;
        public double TARGET_RPM = 5000;
        public double TICKS_PER_REV = 28;
    }
    public static RedFarSpikeAuto.PIDFConfiguration PIDFParams = new RedFarSpikeAuto.PIDFConfiguration();

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
    public static class hood {
        private Servo hood;

        public hood(HardwareMap hardwareMap) {
            hood = hardwareMap.get(Servo.class, "hood2");
        }

        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hood.setPosition((Params.HOOD_SPEED/ 180.0));
                return false;
            }
        }

        public Action up() {
            return new Up();
        }
        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hood.setPosition(-(Params.HOOD_SPEED / 180.0));
                return false;
            }
        }
        public Action down() {
            return new Down();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel flywheel = new flywheel(hardwareMap);
        intake intake = new intake(hardwareMap);
        arm arm = new arm(hardwareMap);
        hood hood = new hood(hardwareMap);

        Pose2d startPose = new Pose2d(61.2, 12.3, Math.toRadians(180));
        Pose2d scorePose = new Pose2d(48, 8, Math.toRadians(160));
        Pose2d thirdLinePose = new Pose2d(32, 23, Math.toRadians(90));
        Pose2d endPose = new Pose2d(45, 35, Math.toRadians(160));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action trajectoryAction = drive.actionBuilder(startPose)
                .stopAndAdd(hood.up())
                .stopAndAdd(flywheel.fly())
                .strafeToLinearHeading(scorePose.position, scorePose.heading)
                .waitSeconds(1.5)
                .stopAndAdd(intake.spin())
                // ball 1
                .stopAndAdd(arm.extend())
                .afterTime(0.1, arm.retract())
                // ball 2
                .afterTime(1.0, intake.pause())
                .afterTime(1.6, arm.extend())
                .afterTime(1.7, arm.retract())
                .afterTime(1.7, intake.spin())
                // ball 3
                .afterTime(3.0, intake.pause())
                .afterTime(3.2, arm.extend())
                .afterTime(3.3, arm.retract())
                .waitSeconds(3.5)
                .stopAndAdd(flywheel.stop())
                .stopAndAdd(intake.spin())
                // SPIKE III
                .splineToLinearHeading(new Pose2d(thirdLinePose.position, thirdLinePose.heading), thirdLinePose.heading)
                .lineToY(70)
                .lineToY(40)
                .stopAndAdd(intake.pause())
                .stopAndAdd(flywheel.fly())
                .strafeToLinearHeading(scorePose.position, Math.toRadians(157.5))
                .stopAndAdd(intake.spin())
                // ball 1
                .stopAndAdd(arm.extend())
                .afterTime(0.1, arm.retract())
                // ball 2
                .afterTime(1.0, intake.pause())
                .afterTime(1.6, arm.extend())
                .afterTime(1.7, arm.retract())
                .afterTime(1.7, intake.spin())
                // ball 3
                .afterTime(3.0, intake.pause())
                .afterTime(3.2, arm.extend())
                .afterTime(3.3, arm.retract())
                .waitSeconds(3.5)
                .stopAndAdd(flywheel.stop())
                .stopAndAdd(hood.down())
                .strafeToLinearHeading(endPose.position, endPose.heading)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(trajectoryAction);




    }
}
