package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.teleop.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Autonomous(name="RedFarAuto", group="comp")
public class RedFarAuto extends LinearOpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Transfer transfer;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);

        Pose2d startPose = new Pose2d(60, 14.5, Math.toRadians(270));
        Pose2d scorePose = new Pose2d(-25, -22, Math.toRadians(230));
        Pose2d firstLinePose = new Pose2d(-9, -21, Math.toRadians(270));
        Pose2d secondLinePose = new Pose2d(15, -24, Math.toRadians(270));
        Pose2d endPose = new Pose2d(60,37,Math.toRadians(270));

        MecanumDriveRR driveRR = new MecanumDriveRR(hardwareMap, startPose);

        Action trajectoryAction = driveRR.actionBuilder(startPose)
                .lineToY(endPose.position.y)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(trajectoryAction);
    }
}
