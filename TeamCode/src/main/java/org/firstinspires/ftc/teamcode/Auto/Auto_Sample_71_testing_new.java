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
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Sequences.FinalAutoSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.Arrays;

@Autonomous(group = "Auto WIP", name = "Auto Sample Specimen Preload")
@Config
@Disabled
@Deprecated
public class Auto_Sample_71_testing_new extends LinearOpMode {
    MecanumDrive drive;

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

    public static double wait1 = 0.5;
    public static double wait2 = 0.5;
    public static double wait3 = 0.85;
    public static double wait4 = 0.7;
    public static double wait5 = 1.3;
    public static double wait6 = 0.65;

    public static Vector2d startpose = new Vector2d(-17,-64);
    public static Vector2d preload_drop = new Vector2d(-4,-40);
    public static Vector2d sample1pick = new Vector2d(-50,-42);
    public static Vector2d sample1drop = new Vector2d(-60,-50);
    public static Vector2d sample2pick = new Vector2d(-59,-42);
    public static Vector2d sample2drop = new Vector2d(-60,-50);
    public static Vector2d sample3pick = new Vector2d(-66,-39.5);
    public static Vector2d sample3drop = new Vector2d(-58,-51);
    public static Vector2d park = new Vector2d(-30,0);



    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(startpose,Math.PI/2));
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();

        Action autoSequence = drive.actionBuilder(new Pose2d(startpose,Math.PI/2))
                ////////////// PRELOAD ////////////////////
                .afterTime(0.01,FinalAutoSeq.SpecimenFrontDropPos(arm,slider))
                .waitSeconds(0.5)
                .strafeToConstantHeading(preload_drop)
                .waitSeconds(0)
                .stopAndAdd(FinalAutoSeq.SpecimenFrontDrop(arm,slider))

//                ////////////// FIRST SAMPLE ////////////////
                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extSpecimenPrePick))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(sample1pick,Math.PI/2),Math.PI)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))

                .waitSeconds(wait4)
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .strafeToLinearHeading(sample1drop,Math.PI/2 - Math.toRadians(15))

                .waitSeconds(wait5)
                .afterTime(0.01,FinalAutoSeq.SampleDrop(arm))
//
//              //////////// SECOND SAMPLE ////////////////
                .waitSeconds(wait2)
                .afterTime(0.1,FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extHighBucketDrop))

                .waitSeconds(wait3)
                .strafeToLinearHeading(sample2pick,Math.PI/2 + Math.toRadians(5))
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))

                .waitSeconds(wait4)
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .strafeToLinearHeading(sample2drop,Math.PI/2 - Math.toRadians(15))

                .waitSeconds(wait5)
                .afterTime(0.01,FinalAutoSeq.SampleDrop(arm))

                //////////// THIRD SAMPLE ////////////////
                .waitSeconds(wait6)
                .afterTime(0.01,FinalAutoSeq.SamplePickPosNoExtLeftYaw(arm,slider,MotorConst.extHighBucketDrop))

                .waitSeconds(wait3)
                .strafeToLinearHeading(sample3pick,Math.PI/2 + Math.toRadians(5))
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))

                .waitSeconds(wait4)
                .afterTime(0.01,FinalAutoSeq.SampleDropPosYawNeutral(arm,slider,MotorConst.extMin,MotorConst.turretDown))
                .strafeToLinearHeading(sample3drop,Math.PI/2  - Math.toRadians(15))

                .waitSeconds(wait5)
                .stopAndAdd(FinalAutoSeq.SampleDrop(arm))

                ///////////////  PARK  ///////////////////
                .waitSeconds(wait6)
                .afterTime(0.01,FinalAutoSeq.TeleOpInit(arm,slider,MotorConst.extHighBucketDrop))
                .strafeToLinearHeading(park, Math.toRadians(90),baseConst)
                .build();


        Action testSeq = drive.actionBuilder(new Pose2d(startpose,Math.PI/2))
                .stopAndAdd(FinalAutoSeq.SpecimenFrontDropPos(arm,slider))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SpecimenFrontDrop(arm,slider))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePickPosNoExtNoYaw(arm,slider,MotorConst.extMin))
                .waitSeconds(2)
                .stopAndAdd(FinalAutoSeq.SamplePick(arm))
                .build();

        if(opModeInInit()) {
            Actions.runBlocking(AutoSeq.SpecimenInit(arm, slider));
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(autoSequence);
//            Actions.runBlocking(testSeq);
        }

    }
}
