package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.CompTeleop;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BlueAutoLeave", group=("comp"))
public class BlueAutoLeave extends LinearOpMode {

    public static CompTeleop.Configuration Params = new CompTeleop.Configuration();

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(61.2, 12.3, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action trajectoryAction = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(61.2, 34), Math.toRadians(180))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(trajectoryAction);




    }
}
