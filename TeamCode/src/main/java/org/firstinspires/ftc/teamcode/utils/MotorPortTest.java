package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Port Test")
@Disabled
public class MotorPortTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class,"m1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class,"m2");
        DcMotorEx m3 = hardwareMap.get(DcMotorEx.class,"m3");
        DcMotorEx m4 = hardwareMap.get(DcMotorEx.class,"m4");

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Port 0",m1.getCurrentPosition());
            telemetry.addData("Port 1",m2.getCurrentPosition());
            telemetry.addData("Port 2",m3.getCurrentPosition());
            telemetry.addData("Port 3",m4.getCurrentPosition());
            telemetry.update();
        }
    }
}
