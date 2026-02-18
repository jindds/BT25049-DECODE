package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotorEx flywheel1, flywheel2;

    public static class PIDFConfiguration {
        public double kP = 70.0;
        public double kI = 0.0;
        public double kD = 20.0;
        public double kF = 15.4;
        public double TARGET_RPM = 3900;
        public double TICKS_PER_REV = 28;
    }

    public static Shooter.PIDFConfiguration PIDFParams = new Shooter.PIDFConfiguration();

    public Shooter(HardwareMap hwMap) {
        flywheel1 = hwMap.get(DcMotorEx.class, "flywheelup");
        flywheel2 = hwMap.get(DcMotorEx.class, "flywheeldown");

        // TODO: reverse dir if needed

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel1.setVelocityPIDFCoefficients(
                PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);
        flywheel2.setVelocityPIDFCoefficients(
                PIDFParams.kP, PIDFParams.kI, PIDFParams.kD, PIDFParams.kF);

    }

    public void fire() {
        double targetTickPerSec =
                (PIDFParams.TARGET_RPM * PIDFParams.TICKS_PER_REV) / 60.0;

            flywheel1.setVelocity(targetTickPerSec);
            flywheel2.setVelocity(targetTickPerSec);
    }

    public void halt() {
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }
}
