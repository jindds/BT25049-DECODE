package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.CompTeleop;

@Config
@TeleOp(name = "Flywheel TEST")
public class testFlywheel extends OpMode {

    public static double kP = 70.0;
    public static double kI = 0.0;
    public static double kD = 20.0;
    public static double kF = 15.4;
    public static double TARGET_RPM = 4500;
    public static double TICKS_PER_REV = 28;
    private boolean flywheelOn;

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    @Override
    public void init() {
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheeldown");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheelup");
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        flywheel2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        flywheelOn = false;

    }

    @Override
    public void loop() {

        flywheel1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        flywheel2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        double targetTickPerSec =
                (TARGET_RPM * TICKS_PER_REV) / 60.0;

        if (gamepad1.a) {
            flywheelOn = true;
        }

        if (flywheelOn) {
            flywheel1.setVelocity(targetTickPerSec);
            flywheel2.setVelocity(targetTickPerSec);
        } else {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }

        double currentRPM =
                (flywheel1.getVelocity() * 60.0) / TICKS_PER_REV;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", TARGET_RPM);
        packet.put("Current RPM", currentRPM);
        packet.put("Velocity Error", TARGET_RPM - currentRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Vel1", flywheel1.getVelocity());
        telemetry.addData("Vel2", flywheel2.getVelocity());
        telemetry.update();
    }
}