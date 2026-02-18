package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public GoBildaPinpointDriver pinpoint;

    public MecanumDrive(HardwareMap hwMap) {

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // TODO: reverse dir of motor

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // TODO: .setoffsets; set encoder directions

        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

    }

    public void drive(double y, double x, double rx, boolean fieldCentric) {
        double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        if (fieldCentric) {
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        frontLeft.setPower((y + x + rx)/ denom);
        frontRight.setPower((y - x - rx)/ denom);
        backRight.setPower((y + x - rx)/ denom);
        backLeft.setPower((y - x + rx)/ denom);
    }

}
