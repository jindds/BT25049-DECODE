package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    public DcMotorEx turret;
    private Limelight3A limelight;
    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0.0;
    private double lastError = 0.0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.4;
    private double power = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update() {
        double deltaTime = timer.seconds();
        timer.reset();

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            turret.setPower(0.0);
            lastError = 0;
            return;
        }

        // PD controller

        double tx = result.getTx();
        double error = goalX - tx;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error-lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // safety encoder check between 0-360

        turret.setPower(power);
        lastError = error;

    }
}
