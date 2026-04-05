package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Config
@TeleOp
public class testPIDF extends OpMode {
    private Shooter shooter;
    private Transfer transfer;

    @Override
    public void init() {
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.fire();
        transfer.gateOpen();
        transfer.startIntake();

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

