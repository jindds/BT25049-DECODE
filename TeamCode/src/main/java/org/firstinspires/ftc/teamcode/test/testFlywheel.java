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

        public static double kP = 200.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 15.4;
        public static double TARGET_RPM = 6000;
        public static double TICKS_PER_REV = 28;
        private boolean flywheelOn;

    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;

    @Override
    public void init() {
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        flywheel2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        flywheelOn = false;

    }

    @Override
    public void loop() {

        flywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        flywheel2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        double targetTickPerSec =
                (TARGET_RPM * TICKS_PER_REV) / 60.0;

        if (gamepad1.a) {
            flywheelOn = true;
        }

        if (flywheelOn) {
            flywheel.setVelocity(targetTickPerSec);
            flywheel2.setVelocity(targetTickPerSec);
        } else {
            flywheel.setPower(0);
            flywheel2.setPower(0);
        }

        double currentRPM =
                (flywheel.getVelocity() * 60.0) / TICKS_PER_REV;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", TARGET_RPM);
        packet.put("Current RPM", currentRPM);
        packet.put("Velocity Error", TARGET_RPM - currentRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.update();
    }
}
