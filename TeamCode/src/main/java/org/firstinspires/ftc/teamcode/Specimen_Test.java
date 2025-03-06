package org.firstinspires.ftc.teamcode;

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

import java.util.Arrays;

@Config
@Autonomous(group = "Auto Specimen", name = "\uD83D\uDD35 Auto Specimen")
//@Disabled
//@Deprecated
public class Specimen_Test extends LinearOpMode {

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

    public static Pose2d start_pose = new Pose2d(new  Vector2d(-17,64),-Math.PI/2);
    public static Vector2d preload_drop = new Vector2d(-5,39);
    public static Vector2d sample1pick = new Vector2d(-31,42);
    public static Vector2d sample1drop = new Vector2d(-31,42.1);
    public static Vector2d sample2pick = new Vector2d(-39,40.5);
    public static Vector2d sample2drop = new Vector2d(-39,40.51);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,start_pose);


        Action autoSequence = drive.actionBuilder(start_pose)
                //PRELOAD SPECIMEN DROP
                .waitSeconds(0.2)
                .strafeToConstantHeading(preload_drop)
                .waitSeconds(0.5)
                //SAMPLE 1 TO OBS
                .strafeToLinearHeading(sample1pick,Math.toRadians(-125))
                .waitSeconds(0.5)
                .strafeToLinearHeading(sample1drop, 3*Math.PI/4)
                .waitSeconds(0.5)
//                //SAMPLE 2 TO OBS
                .strafeToLinearHeading(sample2pick,Math.toRadians(-125))
                .waitSeconds(0.5)
                .strafeToLinearHeading(sample2drop, 3*Math.PI/4)
                .build();

        ////////////////////// TESTING ////////////////////////////


        if(opModeInInit()) {
        }

        waitForStart();
        if(opModeIsActive()){
            Actions.runBlocking(autoSequence);
        }
    }
}
