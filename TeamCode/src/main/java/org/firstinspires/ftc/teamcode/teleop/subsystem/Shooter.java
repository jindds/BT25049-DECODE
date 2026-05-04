package org.firstinspires.ftc.teamcode.teleop.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {
    public DcMotorEx flywheel1, flywheel2;

    public static class PIDFConfiguration {
        public double targetVelocity = 1800;
        public double velocity;
        public double kP = 0.;
        public double kV = 0.000464;
        public double kS = 0.13;
    }

    public static PIDFConfiguration PIDFParams = new PIDFConfiguration();

    public Shooter(HardwareMap hwMap) {
        flywheel1 = hwMap.get(DcMotorEx.class, "flywheelup");
        flywheel2 = hwMap.get(DcMotorEx.class, "flywheeldown");

        // set direction
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void fire() {
        PIDFParams.velocity = flywheel2.getVelocity();

        double error = PIDFParams.targetVelocity - PIDFParams.velocity;
        double feedback = error * PIDFParams.kP;
        double feedforward = PIDFParams.kV * PIDFParams.targetVelocity + PIDFParams.kS;
        // bang bang hybrid
        // if (error >= PIDFParams.targetVelocity * 0.1) {
        //    flywheel1.setPower(1);
        //    flywheel2.setPower(1);
        // } else {
            flywheel1.setPower(feedback + feedforward);
            flywheel2.setPower(feedback + feedforward);
        //}
    }

    public void halt() {
        flywheel1.setPower(0.4);
        flywheel2.setPower(0.4);
    }

    // AUTON ACTIONS
    /* public Action spinUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDFParams.velocity = flywheel2.getVelocity();

                double error = PIDFParams.targetVelocity - PIDFParams.velocity;
                double feedback = error * PIDFParams.kP;
                double feedforward = PIDFParams.kV * PIDFParams.targetVelocity + PIDFParams.kS;
                // bang bang hybrid
                if (error >= PIDFParams.targetVelocity * 0.1) {
                    flywheel1.setPower(1);
                    flywheel2.setPower(1);
                } else {
                    flywheel1.setPower(feedback + feedforward);
                    flywheel2.setPower(feedback + feedforward);
                }
                return false;
            }
        };
    }

    public Action stopSpin() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel1.setPower(0.4);
                flywheel2.setPower(0.4);

                return false;
            }
        };
    }*/
}
