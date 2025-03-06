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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Sequences.CrazySpecimenSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.Arrays;

@Config
@Autonomous(group = "Auto Specimen Testing", name = "\uD83D\uDD35 Auto Specimen Testing")
//@Disabled
public class Auto_Specimen_Blue_Old_Testing extends LinearOpMode {

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

    public static Pose2d start_pose = new Pose2d(new  Vector2d(-17,64),-Math.PI/2);
    public static Vector2d preload_drop = new Vector2d(-5,39);
    public static Vector2d sample1pick = new Vector2d(-33,45);
    public static Vector2d sample1drop = new Vector2d(-33,45.1);
    public static Vector2d sample2pick = new Vector2d(-41,43.5);
    public static Vector2d sample2drop = new Vector2d(-41,43.51);
    public static Vector2d sample3pick = new Vector2d(-49,41);
    public static Vector2d specimen_1pick = new Vector2d(-36,61);
    public static Vector2d specimen_23pick = new Vector2d(-36,64);
    public static Vector2d specimen1drop = new Vector2d(-3,30);
    public static Vector2d specimen2drop = new Vector2d(-1,30);
    public static Vector2d specimen3drop = new Vector2d(1,30);
    public static Vector2d specimen4drop = new Vector2d(3,30);
    public static Vector2d park = new Vector2d(45,-60);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,start_pose);
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        robot.reset_encoders();


        Action autoSequence = drive.actionBuilder(start_pose)
                //PRELOAD SPECIMEN DROP
                .afterTime(0, CrazySpecimenSeq.SpecimenFrontDropPos(arm,slider))
                .waitSeconds(0.2)
                .strafeToConstantHeading(preload_drop)
                .afterTime(0, CrazySpecimenSeq.SpecimenFrontDrop(arm,slider))
                .waitSeconds(0.2)
                //SAMPLE 1 TO OBS
                .afterTime(0, CrazySpecimenSeq.Specimen1PickPos(arm,slider))
                .strafeToLinearHeading(sample1pick,-3*Math.PI/4)
                .afterTime(0, CrazySpecimenSeq.Specimen1Pick_Drop(arm,slider))
                .waitSeconds(0.5)
                .strafeToLinearHeading(sample1drop, 3*Math.PI/4)
                .waitSeconds(0.2)
//                //SAMPLE 2 TO OBS
                .afterTime(0, CrazySpecimenSeq.Specimen2PickPos(arm,slider))
                .strafeToLinearHeading(sample2pick,-3*Math.PI/4)
                .afterTime(0, CrazySpecimenSeq.Specimen2Pick_Drop(arm,slider))
                .waitSeconds(0.5)
                .strafeToLinearHeading(sample2drop, 3*Math.PI/4)
                .waitSeconds(0.45)
//                //SAMPLE 3 TO OBS
                .afterTime(0, CrazySpecimenSeq.Specimen3PickPos(arm,slider))
                .strafeToLinearHeading(sample3pick,-3*Math.PI/4)
                .afterTime(0, CrazySpecimenSeq.Specimen3Pick_Drop(arm,slider))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(specimen_1pick,Math.PI/2),2*Math.PI/3)
//                .waitSeconds(0.3)
//                //FIRST SPECIMEN DROP
                .afterTime(0, CrazySpecimenSeq.Specimen1Scoring(arm,slider))
                .waitSeconds(0.25)
                .setReversed(true)
                .splineToConstantHeading(specimen1drop,3*Math.PI/2)
                .waitSeconds(0.15)
                .splineToConstantHeading(specimen_23pick,Math.PI/2)
//                //SECOND SPECIMEN DROP
                .afterTime(0, CrazySpecimenSeq.Specimen2Scoring(arm,slider))
                .waitSeconds(0.25)
                .setReversed(true)
                .splineToConstantHeading(specimen2drop,3*Math.PI/2)
                .waitSeconds(0.15)
                .splineToConstantHeading(specimen_23pick,Math.PI/2)
//                //THIRD SPECIMEN DROP
                .afterTime(0, CrazySpecimenSeq.Specimen3Scoring(arm,slider))
                .waitSeconds(0.25)
                .setReversed(true)
                .splineToConstantHeading(specimen3drop,3*Math.PI/2)
                .waitSeconds(0.15)
                .splineToConstantHeading(specimen_23pick,Math.PI/2)
//                //FOURTH SPECIMEN DROP
                .afterTime(0, CrazySpecimenSeq.Specimen4Scoring(arm,slider))
                .waitSeconds(0.25)
                .setReversed(true)
                .splineToConstantHeading(specimen4drop,3*Math.PI/2)
                .waitSeconds(0.15)
                //PARKING
                .afterTime(0,AutoSeq.TeleOpInit(arm,slider))
                .strafeToConstantHeading(park,baseConst)
                .waitSeconds(3)
                .build();

        ////////////////////// TESTING ////////////////////////////
        Action testSeq = drive.actionBuilder(start_pose)
                .stopAndAdd(CrazySpecimenSeq.SpecimenFrontDropPos(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.SpecimenFrontDrop(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen1PickPos(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen1Pick_Drop(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen2PickPos(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen2Pick_Drop(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen3PickPos(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen3Pick_Drop(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen1Scoring(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen2Scoring(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen3Scoring(arm,slider))
                .waitSeconds(1.5)
                .stopAndAdd(CrazySpecimenSeq.Specimen4Scoring(arm,slider))
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
