package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Config
@TeleOp(name="BT-25049-Teleop", group="comp")
public class CompTeleop extends OpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Transfer transfer;
    private boolean FieldCentric, flywheelReady, gateReady = false;
    private boolean prevA, prevB, prevY, prevLB, prevRB = false;
    enum IntakeState {
        OFF,
        FORWARD,
        REVERSE
    }
    IntakeState intakeState = IntakeState.OFF;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        transfer.gateClose();
    }

    @Override
    public void loop() {
        // DRIVE
        drive.pinpoint.update();
        drive.drive(
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, FieldCentric);
        if (gamepad1.a && !prevA) {
            FieldCentric = !FieldCentric;
        }
        if (gamepad1.x) {
            drive.pinpoint.recalibrateIMU();
            drive.pinpoint.resetPosAndIMU();
        }
        prevA = gamepad1.a;

        // INTAKE OR STOP
        if (gamepad1.right_bumper && !prevRB) {
            if (intakeState == IntakeState.FORWARD) {
                intakeState = IntakeState.OFF; // turn off when already forward
            } else {
                intakeState = IntakeState.FORWARD; // otherwise, run forward
            }
        }

        // OUTTAKE OR STOP
        if (gamepad1.left_bumper && !prevLB) {
            if (intakeState == IntakeState.REVERSE) {
                intakeState = IntakeState.OFF; // turn off when already reverse
            } else {
                intakeState = IntakeState.REVERSE; // otherwise, run reverse
            }
        }

        prevLB = gamepad1.left_bumper;
        prevRB = gamepad1.right_bumper;

        // ACTIVATE TRANSFER
        switch(intakeState) {
            case FORWARD:
                transfer.startIntake();
                break;
            case REVERSE:
                transfer.startOuttake();
                break;
            default:
                transfer.stopTransfer();
                break;
        }

        // GATE
        //if (gamepad1.y && !prevY) {
          //  gateReady = !gateReady;
        //}
        //prevY = gamepad1.y;

        //if (gateReady) {
          //  transfer.gateOpen();
        //} else {
          //  transfer.gateClose();
        //}

        // SHOOT
        if (gamepad1.b && !prevB) {
            flywheelReady = !flywheelReady;
            gateReady = !gateReady;
        }
        prevB = gamepad1.b;

        if (flywheelReady) {
            shooter.fire();
            transfer.gateOpen();
        } else {
            shooter.halt();
            transfer.gateClose();
        }

        // TELEMETRY
        telemetry.addData("curVel", shooter.flywheel2.getVelocity());
        telemetry.addData("TargetVel", Shooter.PIDFParams.targetVelocity);
        telemetry.addData("CurrentVel", Shooter.PIDFParams.velocity);
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        packet.put("TargetVel", Shooter.PIDFParams.targetVelocity);
        packet.put("CurrentVel", Shooter.PIDFParams.velocity);
        packet.put("error", Shooter.PIDFParams.targetVelocity - Shooter.PIDFParams.velocity);

    }
}