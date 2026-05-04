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

@Autonomous(name = "Blue 12 #2", group = "Autonomous")
@Configurable // Panels
public class Blue12Auto2 extends OpMode {
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
        follower.setStartingPose(new Pose(33.346, 136.075, Math.toRadians(180)));

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
        public PathChain clearRamp;
        public PathChain firstShootPose;
        public PathChain firstShoot;
        public PathChain secondPickup;
        public PathChain secondShootPose;
        public PathChain secondShoot;
        public PathChain thirdPickup;
        public PathChain thirdShootPose;
        public PathChain thirdShoot;

        public Paths(Follower follower) {
            preLoad = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.346, 136.075),

                                    new Pose(33.346, 110.467)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            firstPickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(33.346, 110.467),
                                    new Pose(41.047, 81.794),
                                    new Pose(16.523, 82.523)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            clearRamp = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.523, 82.523),
                                    new Pose(23.257, 78.425),
                                    new Pose(16.271, 73.505)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            firstShootPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.271, 73.505),

                                    new Pose(33.112, 110.280)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            firstShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.112, 110.280),

                                    new Pose(33.112, 110.281)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            secondPickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(33.112, 110.281),
                                    new Pose(45.201, 57.682),
                                    new Pose(8.916, 57.421)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            secondShootPose = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.916, 57.421),
                                    new Pose(25.262, 59.944),
                                    new Pose(33.159, 110.542)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            secondShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.159, 110.542),

                                    new Pose(33.159, 110.543)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();


            thirdPickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(33.159, 110.543),
                                    new Pose(41.126, 28.182),
                                    new Pose(28.687, 38.051),
                                    new Pose(9.841, 36.869)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            thirdShootPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.841, 36.869),

                                    new Pose(38.346, 124.636)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()

                    .build();

            thirdShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(38.346, 124.636),

                                    new Pose(38.346, 124.637)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(155))
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
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    shooter.halt();
                    transfer.gateClose();
                    follower.followPath(paths.firstPickup, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    transfer.stopTransfer();
                    follower.followPath(paths.clearRamp, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.firstShootPose, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    transfer.startIntake();
                    follower.followPath(paths.firstShoot, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shooter.fire();
                    transfer.gateOpen();
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
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
                    follower.followPath(paths.secondShoot, true);
                    setPathState(10);
                }
            case 10:
                if (!follower.isBusy()) {
                    shooter.fire();
                    transfer.gateOpen();
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    shooter.halt();
                    transfer.gateClose();
                    follower.followPath(paths.thirdPickup, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.thirdShootPose, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    shooter.fire();
                    transfer.gateOpen();
                }
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
