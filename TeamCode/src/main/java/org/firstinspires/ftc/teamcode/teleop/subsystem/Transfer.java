package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Transfer {
    public DcMotor intake, highIntake;
    public Servo gate;

    public static class Configuration {
        public double INTAKE_SPEED = 1.0;
        public double startPosGate = 0.35;
        public double openPosGate = 0;

    }

    public static Configuration Params = new Configuration();

    public Transfer(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        highIntake = hwMap.get(DcMotor.class, "highIntake");
        gate = hwMap.get(Servo.class, "gate");

        highIntake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void startIntake() {
        intake.setPower(Params.INTAKE_SPEED);
        highIntake.setPower(-1);
    }
    public void startOuttake() {
        intake.setPower(-Params.INTAKE_SPEED);
        highIntake.setPower(1);
    }
    public void stopTransfer() {
        intake.setPower(0);
        highIntake.setPower(0);
    }
    public void gateOpen() { gate.setPosition(Params.openPosGate); }
    public void gateClose() { gate.setPosition(Params.startPosGate); }
}
