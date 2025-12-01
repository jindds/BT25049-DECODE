package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class testServo extends OpMode {

    public static class Configuration {
        public double SERVO_RANGE_DEGREES = 100.0;
    }

    public static testServo.Configuration Params = new testServo.Configuration();
    Servo servo;

    public void init() {
        servo = hardwareMap.get(Servo.class, "arm");
    }
    public void loop() {
        if (gamepad1.a) {
            double targetPosition = Params.SERVO_RANGE_DEGREES / 180.0;
            servo.setPosition(targetPosition);
        } else {
            servo.setPosition(- (Params.SERVO_RANGE_DEGREES / 180.0));
            }

    }




}
