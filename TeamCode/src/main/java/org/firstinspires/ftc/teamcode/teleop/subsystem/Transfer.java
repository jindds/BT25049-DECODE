package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public DcMotor intake;
    public Servo gate;

    public static class Configuration {
        public double INTAKE_SPEED = 0.85;
        public double startPosGate = 0.0; // TODO: change start and close gate values
        public double openPosGate = 0.0;
    }

    public static Transfer.Configuration Params = new Transfer.Configuration();

    public Transfer(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        gate = hwMap.get(Servo.class, "gate");

        // TODO: reverse dir of motors if needed
    }

    public void startIntake() { intake.setPower(Params.INTAKE_SPEED); }
    public void startOuttake() { intake.setPower(-Params.INTAKE_SPEED); }
    public void stopTransfer() { intake.setPower(0); }
    public void gateOpen() { gate.setPosition(Params.openPosGate); }
    public void gateClose() { gate.setPosition(Params.startPosGate); }
}
