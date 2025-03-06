package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Servo Port Test")
public class ServoPortTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class,"s1");
        Servo s2 = hardwareMap.get(Servo.class,"s2");
        Servo s3 = hardwareMap.get(Servo.class,"s3");
        Servo s4 = hardwareMap.get(Servo.class,"s4");
        Servo s5 = hardwareMap.get(Servo.class,"s5");

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.a) {
                s1.setPosition(0.5);
                sleep(200);
                s1.setPosition(0.4);
                sleep(200);
                s1.setPosition(0.6);
                sleep(200);
                s1.setPosition(0.5);
                telemetry.addLine("Servo port 0 running");
                telemetry.update();
            }

            if(gamepad1.b) {
                s2.setPosition(0.5);
                sleep(200);
                s2.setPosition(0.4);
                sleep(200);
                s2.setPosition(0.6);
                sleep(200);
                s2.setPosition(0.5);
                telemetry.addLine("Servo port 1 running");
                telemetry.update();
            }

            if(gamepad1.x) {
                s3.setPosition(0.5);
                sleep(200);
                s3.setPosition(0.4);
                sleep(200);
                s3.setPosition(0.6);
                sleep(200);
                s3.setPosition(0.5);
                telemetry.addLine("Servo port 2 running");
                telemetry.update();
            }

            if(gamepad1.y) {
                s4.setPosition(0.5);
                sleep(200);
                s4.setPosition(0.4);
                sleep(200);
                s4.setPosition(0.6);
                sleep(200);
                s4.setPosition(0.5);
                telemetry.addLine("Servo port 3 running");
                telemetry.update();
            }

            if(gamepad1.dpad_up) {
                s1.setPosition(0.5);
                sleep(200);
                s1.setPosition(0.4);
                sleep(200);
                s1.setPosition(0.6);
                sleep(200);
                s1.setPosition(0.5);
                telemetry.addLine("Servo port 4 running");
                telemetry.update();
            }
        }
    }
}
