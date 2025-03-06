package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //DRIVE
    DcMotorEx leftFront,leftBack,rightFront,rightBack;

    //SLIDER
    public DcMotorEx extRight,extLeft,turret;

    //ARM
    public Servo shoulder,yaw,elbow,wrist,gripper;

    //TODO HANGER
    public DcMotorEx hanger;

    //Todo Color Sensor, Camera
    public RevColorSensorV3 colorSensor;

    public RobotHardware(HardwareMap hardwareMap){

        //SLIDER init
        this.extRight = hardwareMap.get(DcMotorEx.class,"extensionRight");
        this.extRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.extLeft = hardwareMap.get(DcMotorEx.class,"extensionLeft");
        this.extLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turret = hardwareMap.get(DcMotorEx.class,"turret");
        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turret.setDirection(DcMotorSimple.Direction.REVERSE);

        //ARM init
        this.shoulder = hardwareMap.get(Servo.class,"shoulder");
        this.yaw = hardwareMap.get(Servo.class,"yaw");
        this.elbow = hardwareMap.get(Servo.class,"elbow");
        this.wrist = hardwareMap.get(Servo.class,"wrist");
        this.gripper = hardwareMap.get(Servo.class,"gripper");

        //HANGER init
        this.hanger = hardwareMap.get(DcMotorEx.class,"hanger");
        this.hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hanger.setDirection(DcMotorSimple.Direction.REVERSE);

        //Color Sensor
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class,"color");
        this.colorSensor.setGain(50);
    }

    public void init_encoders(){
        extRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extLeft.setTargetPositionTolerance(10);
        extRight.setTargetPositionTolerance(10);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset_encoders(){
        extRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
