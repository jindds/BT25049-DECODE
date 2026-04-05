package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Transfer;

@Autonomous(name="RedCloseAuto", group="comp")

public class RedCloseAuto extends LinearOpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Transfer transfer;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);

        transfer.gateClose();

        waitForStart();

        backward(600);
        Shooter.PIDFParams.velocity = shooter.flywheel2.getVelocity();
            shooter.fire();
            transfer.gateOpen();
            sleep(1000);
            transfer.intake.setPower(-0.9);
            transfer.highIntake.setPower(-1.0);
            sleep(5000);
            transfer.gateClose();


    }

    private void backward(int time) {
        drive.frontLeft.setPower(-0.5);
        drive.frontRight.setPower(-0.5);
        drive.backLeft.setPower(-0.5);
        drive.backRight.setPower(-0.5);

        sleep(time);

        drive.frontLeft.setPower(0);
        drive.frontRight.setPower(0);
        drive.backLeft.setPower(0);
        drive.backRight.setPower(0);

    }
}