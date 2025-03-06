package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.FinalAutoSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.Arrays;

@Config
@Autonomous(group = "Auto WIP", name = "Auto new")
@Deprecated
@Disabled
public class Auto_new extends LinearOpMode {

    MecanumDrive drive;

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

    public static double wait1 = 0.5;
    public static double wait2 = 0.3;
    public static double wait3 = 0.8;
    public static double wait4 = 0.7;
    public static double wait5 = 1;
    public static double wait6 = 0.3;

    public static Vector2d sample1drop = new Vector2d(-50,-55);
    public static Vector2d sample2pick = new Vector2d(11,-57);
    public static Vector2d sample2drop = new Vector2d(-50,-55);
    public static Vector2d sample3pick = new Vector2d(-49,-37);
    public static Vector2d sample3drop = new Vector2d(-59,-48.5);
    public static Vector2d sample4pick = new Vector2d(-59,-37);
    public static Vector2d sample4drop = new Vector2d(-57.5,-47);
    public static Vector2d sample5pick = new Vector2d(-66,-35);
    public static Vector2d sample5drop = new Vector2d(-57.5,-47);
    public static Vector2d park = new Vector2d(-30,0);



    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(-40,-60),0));
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();

        Action autoSequence = drive.actionBuilder(new Pose2d(new Vector2d(-40,-60),0))
                ///////////////////// FIRST SAMPLE ////////////////////////
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawRight(arm,slider,MotorConst.extInit,MotorConst.turretInit))
                .strafeToConstantHeading(sample1drop)
                //TODO WAIT ADJUST
                .waitSeconds(wait1)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                ///////////////////// SECOND SAMPLE ///////////////////////
//                .afterTime(0.3,FinalAutoSeq.SamplePickPosFullExtRightYaw(arm,slider))
//                .strafeToLinearHeading(sample2pick,0)
//                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
//                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawRight(arm,slider))
//                .strafeToLinearHeading(sample2drop,0)
//                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                //TODO WAIT ADJUST
                .waitSeconds(wait2)
                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extHighBucketDrop))
                //TODO WAIT ADJUST
                .waitSeconds(wait3)
                .strafeToLinearHeading(sample3pick,Math.PI/2 + Math.toRadians(4))
//                ////////////////////// THIRD SAMPLE ///////////////////////
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                //TODO WAIT ADJUST
                .waitSeconds(wait4)
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawLeft(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .strafeToConstantHeading(sample3drop)
                //TODO WAIT ADJUST
                .waitSeconds(wait5)
                .stopAndAdd(FinalAutoSeq.SampleSideDrop(arm))
                //TODO WAIT ADJUST
                .waitSeconds(wait2)
                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extHighBucketDrop))
//                //TODO WAIT ADJUST
                .waitSeconds(wait3)
                .strafeToLinearHeading(sample4pick,Math.PI/2 + Math.toRadians(4))
////                //////////////////////// FOURTH ELEMENT /////////////////////
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                //TODO WAIT ADJUST
                .waitSeconds(wait4)
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawLeft(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .strafeToConstantHeading(sample4drop)
                //TODO WAIT ADJUST
                .waitSeconds(wait5)
                .stopAndAdd(FinalAutoSeq.SampleSideDrop(arm))
                //TODO WAIT ADJUST
                .waitSeconds(wait6)
                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtLeftYaw(arm,slider,MotorConst.extHighBucketDrop))
//                //TODO WAIT ADJUST
                .waitSeconds(wait3)
                .strafeToLinearHeading(sample5pick,Math.PI/2 + Math.toRadians(3))
////                //////////////////////// FIFTH ELEMENT //////////////////////
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
//                //TODO WAIT ADJUST
                .waitSeconds(wait4)
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawLeft(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .strafeToConstantHeading(sample5drop)
//                //TODO WAIT ADJUST
                .waitSeconds(wait5)
                .stopAndAdd(FinalAutoSeq.SampleSideDrop(arm))
//                //TODO WAIT ADJUST
                .waitSeconds(wait6)
                .afterTime(0.01,FinalAutoSeq.TeleOpInit(arm,slider,MotorConst.extHighBucketDrop))
                .strafeToConstantHeading(park,baseConst)
                .build();

        Action testSeq = drive.actionBuilder(new Pose2d(new Vector2d(-40,-60),0))
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extInit,MotorConst.turretInit))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosFullExtRightYaw(arm,slider,MotorConst.extHighBucketDrop,MotorConst.turretUp))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extHorizontalMax,MotorConst.turretDown))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extHighBucketDrop))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosNoExtLeftYaw(arm,slider,MotorConst.extHighBucketDrop))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extHorizontalMax,MotorConst.turretDown))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.TeleOpInit(arm,slider,MotorConst.extHighBucketDrop))
                .build();

        if(opModeInInit()) {
            Actions.runBlocking(FinalAutoSeq.Init(arm, slider));
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(autoSequence);
        }


    }
}
