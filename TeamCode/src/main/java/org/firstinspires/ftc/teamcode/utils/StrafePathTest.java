package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(group = "Path Test", name = "Strafe Path Test")
public class StrafePathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));

        Action t1 = drive.actionBuilder(new Pose2d(new Vector2d(0,0),0))
                .strafeTo(new Vector2d(0,40))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0,0))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0,40))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0,0))
                .build();

        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(t1);
        }
    }
}
