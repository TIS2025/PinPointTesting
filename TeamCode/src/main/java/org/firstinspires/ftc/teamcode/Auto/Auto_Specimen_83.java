package org.firstinspires.ftc.teamcode.Auto;

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

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AutoSeq;
import org.firstinspires.ftc.teamcode.Sequences.FinalSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.Arrays;

@Autonomous(group = "Auto", name = "Auto Specimen 1 + 3")
@Disabled
@Deprecated
public class Auto_Specimen_83 extends LinearOpMode {

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

    public static Pose2d start_pose = new Pose2d(new  Vector2d(17,-64),-Math.PI/2);
    public static Vector2d preload_drop = new Vector2d(2,-31);
    public static Vector2d sample1pick = new Vector2d(30.5,-31.5);
    public static Vector2d sample1drop = new Vector2d(32,-47);
    public static Vector2d sample2pick = new Vector2d(41.5,-29.5);
    public static Vector2d sample2drop = new Vector2d(38,-47);
    public static Vector2d sample3pick = new Vector2d(51.5,-29);
    public static Vector2d sample3drop = new Vector2d(42,-47);
    public static Vector2d specimen_pick = new Vector2d(36,-64);
    public static Vector2d specimen1drop = new Vector2d(5,-30);
    public static Vector2d specimen2drop = new Vector2d(8,-30);
    public static Vector2d specimen3drop = new Vector2d(11,-30);
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
                .afterTime(0.01,AutoSeq.SpecimenDropPos(arm,slider))
                .strafeToConstantHeading(preload_drop)
                .stopAndAdd(AutoSeq.SpecimenDrop(arm,slider))
                //SAMPLE 1 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromSpecimen(arm,slider))
                .afterTime(1.8,AutoSeq.SamplePickForSpecimen(arm,slider))
                .splineToLinearHeading(new Pose2d(sample1pick,Math.PI/12),Math.PI/4)
                .waitSeconds(0.25)
                .afterTime(0.7,AutoSeq.SampleDropObsZone(arm,slider))
                .strafeToLinearHeading(sample1drop, -Math.PI/4)
                //SAMPLE 2 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromObs(arm,slider))
                .afterTime(0.95,AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(sample2pick,Math.PI/12)
                .waitSeconds(0.25)
                .afterTime(0.75,AutoSeq.SampleDropObsZone(arm,slider))
                .strafeToLinearHeading(sample2drop, -Math.PI/4)
                //SAMPLE 3 TO OBS
                .afterTime(0.01,AutoSeq.SamplePickPosFromObs(arm,slider))
                .afterTime(0.95,AutoSeq.SamplePickForSpecimen(arm,slider))
                .strafeToLinearHeading(sample3pick,Math.PI/12)
                .waitSeconds(0.25)
                .afterTime(0.65,AutoSeq.SampleDropObsZone(arm,slider))
                .strafeToLinearHeading(sample3drop, -Math.PI/4)
//                FIRST SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPosFromSampleDrop(arm,slider))
                .waitSeconds(0.45)
                .splineToLinearHeading(new Pose2d(specimen_pick,-Math.PI/2),3*Math.PI/2)
                .stopAndAdd(FinalSeq.SpecimenPick(arm,slider))
                .setReversed(true)
                .splineToConstantHeading(specimen1drop,Math.PI/2)
                .stopAndAdd(FinalSeq.SpecimenDrop(arm,slider))
                //SECOND SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPos(arm,slider))
                .splineToConstantHeading(specimen_pick,-Math.PI/2)
                .stopAndAdd(FinalSeq.SpecimenPick(arm,slider))
                .setReversed(true)
                .splineToConstantHeading(specimen2drop,Math.PI/2)
                .stopAndAdd(FinalSeq.SpecimenDrop(arm,slider))
                //THIRD SPECIMEN DROP
                .afterTime(0.01,AutoSeq.SpecimenPickPos(arm,slider))
                .splineToConstantHeading(specimen_pick,-Math.PI/2)
                .stopAndAdd(FinalSeq.SpecimenPick(arm,slider))
                .setReversed(true)
                .splineToConstantHeading(specimen3drop,Math.PI/2)
                .stopAndAdd(FinalSeq.SpecimenDrop(arm,slider))
                //PARKING
                .afterTime(0.01,AutoSeq.TeleOpInit(arm,slider))
                .strafeToConstantHeading(park,baseConst)
                .build();



        if(opModeInInit()) {
            Actions.runBlocking(AutoSeq.SpecimenInit(arm, slider));
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(autoSequence);
        }
    }
}
