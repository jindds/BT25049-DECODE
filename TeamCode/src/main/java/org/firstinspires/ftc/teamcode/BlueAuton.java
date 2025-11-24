package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="BT-25049-BlueAutonomous-high", group="comp-high-battery")
public class BlueAuton extends LinearOpMode{

    public static CompTeleop.Configuration Params = new CompTeleop.Configuration();

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
        }

        public class Fly implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel.setPower(Params.FLYWHEEL_SPEED-0.05);
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

        Pose2d startPose = new Pose2d(-51,-48, Math.toRadians(230));
        Pose2d scorePose = new Pose2d(-28, -23,Math.toRadians(230));
        Pose2d firstLinePose = new Pose2d(-12, -21, Math.toRadians(270));
        Pose2d secondLinePose = new Pose2d(12, -24, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action trajectoryAction = drive.actionBuilder(startPose)
                .stopAndAdd(flywheel.fly())
                .strafeToLinearHeading(scorePose.position, scorePose.heading)
                // ball 1
                .stopAndAdd(arm.extend())
                .afterTime(0.1, arm.retract())
                // ball 2
                .stopAndAdd(intake.spin())
                .afterTime(1.1, intake.pause())
                .afterTime(1.2, arm.extend())
                .afterTime(2.0, arm.retract())
                // ball 3
                .afterTime(2.1, intake.spin())
                .afterTime(3.1, intake.pause())
                .afterTime(3.2, arm.extend())
                .afterTime(4.0, arm.retract())
                .waitSeconds(4)
                .stopAndAdd(flywheel.stop())
                .stopAndAdd(intake.spin())
                // SPIKE I
                .splineToLinearHeading(new Pose2d(firstLinePose.position, firstLinePose.heading), firstLinePose.heading)
                .lineToY(-55)
                .stopAndAdd(intake.pause())
                .stopAndAdd(flywheel.fly())
                .splineToLinearHeading(new Pose2d(scorePose.position, scorePose.heading), scorePose.heading)
                // ball 1
                .stopAndAdd(arm.extend())
                .afterTime(0.1, arm.retract())
                // ball 2
                .stopAndAdd(intake.spin())
                .afterTime(1.1, intake.pause())
                .afterTime(1.2, arm.extend())
                .afterTime(2.0, arm.retract())
                // ball 3
                .afterTime(2.1, intake.spin())
                .afterTime(3.1, intake.pause())
                .afterTime(3.2, arm.extend())
                .afterTime(4.0, arm.retract())
                .waitSeconds(4)
                .stopAndAdd(flywheel.stop())
                .stopAndAdd(intake.spin())
                // SPIKE II
                .splineToLinearHeading(new Pose2d(secondLinePose.position, secondLinePose.heading), secondLinePose.heading)
                .lineToY(-55)
                .lineToY(-40)
                .stopAndAdd(intake.pause())
                .stopAndAdd(flywheel.fly())
                .splineToLinearHeading(new Pose2d(scorePose.position, scorePose.heading), scorePose.heading)
                // ball 1
                .stopAndAdd(arm.extend())
                .afterTime(0.1, arm.retract())
                // ball 2
                .stopAndAdd(intake.spin())
                .afterTime(1.1, intake.pause())
                .afterTime(1.2, arm.extend())
                .afterTime(2.0, arm.retract())
                // ball 3
                .afterTime(2.1, intake.spin())
                .afterTime(3.1, intake.pause())
                .afterTime(3.2, arm.extend())
                .afterTime(4.0, arm.retract())
                .waitSeconds(4)
                .stopAndAdd(flywheel.stop())
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(trajectoryAction);

    }

}
