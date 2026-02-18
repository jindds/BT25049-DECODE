package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystem.TurretMechanism;

public class testTurret extends OpMode {
    private TurretMechanism turret = new TurretMechanism();

    @Override
    public void init() {
        turret.init(hardwareMap);
    }

    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {
        turret.update();
    }
}
