package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMU Test")
//@Disabled
public class IMUTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        IntegratingGyroscope gyro;
        NavxMicroNavigationSensor navxSensor = hardwareMap.get(NavxMicroNavigationSensor.class,"navx");
//        gyro = (IntegratingGyroscope)navxSensor;
        navxSensor.initialize();

        waitForStart();
        while (opModeIsActive()){
            Orientation orientation = navxSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES);
            telemetry.addData("IMU 1st",orientation.firstAngle);
            telemetry.addData("IMU 2nd",orientation.secondAngle);
            telemetry.addData("IMU 3rd",orientation.thirdAngle);
            telemetry.update();
        }
    }
}
