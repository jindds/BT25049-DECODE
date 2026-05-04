package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Autonomous(name = "Blue 9 + 3 CLEAR", group = "Autonomous")
@Configurable // Panels
public class Blue9Auto extends OpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Transfer transfer;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.252, 114.617, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0); // set initial path state
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }


    public class Paths {
        public PathChain preLoad;
        public PathChain firstPickup;
//        public PathChain clearRamp;
        public PathChain firstShootPose;
        public PathChain secondPickup;
        public PathChain secondShootPose;
        public PathChain thirdClear;
        public PathChain park;

        public Paths(Follower follower) {
            // lower gate start pos
            preLoad = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.252, 114.617),

                                    new Pose(33.346, 110.467)

                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .build();

            firstPickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.346, 110.467),

                                    new Pose(42.701, 85.738)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .addPath(
                            new BezierLine(
                                    new Pose(42.701, 85.738),

                                    new Pose(15.523, 83.645)
                            )
                    ).setTangentHeadingInterpolation()

                    // clear ramp
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.523, 83.645),
                                    new Pose(23.257, 78.425),
                                    new Pose(14.271, 73.505)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();


            firstShootPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.271, 73.505),

                                    new Pose(37.832, 106.430)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()

                    .build();

            secondPickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(37.832, 106.430),

                                    new Pose(44.720, 59.458)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .addPath(
                            new BezierLine(
                                    new Pose(44.720, 59.458),

                                    new Pose(8.019, 58.991)
                            )
                    ).setTangentHeadingInterpolation()

                    // clear ramp
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.019, 58.991),
                                    new Pose(29.013, 60.500),
                                    new Pose(14.400, 73.505)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            secondShootPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.400, 73.505),
                                    new Pose(37.832, 106.430)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()

                    .build();

            thirdClear = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(37.832, 106.430),

                                    new Pose(14.400, 73.505)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.400, 73.505),

                                    new Pose(25.243, 73.505)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                transfer.startIntake();
                follower.followPath(paths.preLoad);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooter.fire();
                    transfer.gateOpen();
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTime() > 1750) {
                    shooter.halt();
                    transfer.gateClose();
                    follower.followPath(paths.firstPickup, true);
                    setPathState(3);
                }
                break;
/*
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.clearRamp, true);
                    setPathState(4);
                }
                break;
*/
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(paths.firstShootPose, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(new Pose(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            Math.toRadians(135)
                    ));
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (actionTimer.getElapsedTime() > 1250) {
                    shooter.fire();
                    transfer.gateOpen();
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTime() > 1750) {
                    shooter.halt();
                    transfer.gateClose();
                    follower.followPath(paths.secondPickup, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.secondShootPose, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.holdPoint(new Pose(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            Math.toRadians(135)
                    ));
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if (actionTimer.getElapsedTime() > 1250) {
                    shooter.fire();
                    transfer.gateOpen();
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (actionTimer.getElapsedTime() > 1750) {
                    shooter.halt();
                    transfer.gateClose();
                    follower.followPath(paths.thirdClear);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.park);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
         }
    }

    // These change the states of the paths and actions. It will also reset the timers of the individual switches
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setActionTimer() {

    }
}
