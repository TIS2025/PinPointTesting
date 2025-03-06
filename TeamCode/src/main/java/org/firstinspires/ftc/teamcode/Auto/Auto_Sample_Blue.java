package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.CrazySampleSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.VisionUtils.LimeLightUtils;

import java.util.Arrays;

@Autonomous(group = "Auto Sample", name = "\uD83D\uDD35 Auto Sample")
@Config
public class Auto_Sample_Blue extends LinearOpMode {
    MecanumDrive drive;

    LimeLightUtils limeUtils;

    VelConstraint baseConst = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(100)));
     AccelConstraint baseAccelConstraint2 = new ProfileAccelConstraint(-45, 90);
    AccelConstraint baseAccelConstraint3 = new ProfileAccelConstraint(-45, 90);
    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(90),
            new AngularVelConstraint(Math.PI / 2)
    ));
//

    public static Pose2d startpose = new Pose2d(new Vector2d(-41,-64),Math.PI/2);
    public static Vector2d preload_drop = new Vector2d(-61,-54);
    public static Vector2d sample1pick = new Vector2d(-53,-38.5);
    public static Vector2d sample1drop = new Vector2d(-63,-53);
    public static Vector2d sample2pick = new Vector2d(-58,-39);
    public static Vector2d sample2drop = new Vector2d(-64,-52);
    public static Vector2d sample3pick = new Vector2d(-65,-38);
    public static Vector2d sample3drop = new Vector2d(-64,-52);
    public static Vector2d sampleLL1pick = new Vector2d(-25,-12);
    public static Vector2d sampleLL1drop = new Vector2d(-55.5,-55.5);
    public static Vector2d park = new Vector2d(-30,0);
    Action lime_pick;

    static enum AutoState{
        ONE_PLUS_3,
        LL_PICK1,
        DROP_N_RETURN,
        LL_PICK2,
        DROP_N_PARK
    }

    AutoState autoState = AutoState.ONE_PLUS_3;


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardwareMap,startpose);
//        drive = new MecanumDrive(hardwareMap,new Pose2d(sampleLL1pick,Math.toRadians(0)));
        limeUtils = new LimeLightUtils(hardwareMap);
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Arm arm = new Arm(robot);
        limeUtils.limelight3A.pipelineSwitch(0);
        robot.reset_encoders();

        Thread limelight_thread = new Thread(() -> {
            while (opModeIsActive()) {
                limeUtils.clear_samples();
                limeUtils.filter(limeUtils.limelight3A.getLatestResult(), telemetry);
                synchronized (this) {
                    update_target_sample();
                }
                limeUtils.print_samples(telemetry);
                telemetry.update();
            }
        });

        Action one_plus_three = drive.actionBuilder(startpose)
                ////////////// PRELOAD ////////////////////
                .afterTime(0,CrazySampleSeq.PreloadDropPos(arm,slider))
                .strafeToLinearHeading(preload_drop, Math.toRadians(70))
                .waitSeconds(0.25)
                .stopAndAdd(CrazySampleSeq.GripperOpen(arm))
//              ////////////// FIRST SAMPLE ////////////////
                .afterTime(0,CrazySampleSeq.Sample1PickPos(arm,slider))
                .strafeToConstantHeading(sample1pick)
                .waitSeconds(0.95)
                .stopAndAdd(CrazySampleSeq.GripperClose(arm))
                .afterTime(0,CrazySampleSeq.Sample1DropPos(arm,slider))
                .strafeToLinearHeading(sample1drop,Math.toRadians(85))
                .waitSeconds(0.9)
                .stopAndAdd(CrazySampleSeq.GripperOpen(arm))
//              //////////// SECOND SAMPLE ////////////////
                .afterTime(0,CrazySampleSeq.Sample2PickPos(arm,slider))
                .strafeToConstantHeading(sample2pick)
                .waitSeconds(0.95)
                .stopAndAdd(CrazySampleSeq.GripperClose(arm))
                .afterTime(0,CrazySampleSeq.Sample2DropPos(arm,slider))
                .strafeToLinearHeading(sample2drop,Math.toRadians(90))
                .waitSeconds(1)
                .stopAndAdd(CrazySampleSeq.GripperOpen(arm))
                //////////// THIRD SAMPLE ////////////////
                .afterTime(0,CrazySampleSeq.Sample3PickPos(arm,slider))
                .strafeToConstantHeading(sample3pick)
                .waitSeconds(0.95)
                .stopAndAdd(CrazySampleSeq.GripperClose(arm))
                .afterTime(0,CrazySampleSeq.Sample3DropPos(arm,slider))
                .strafeToConstantHeading(sample3drop)
                .waitSeconds(0.9)
                .stopAndAdd(CrazySampleSeq.GripperOpen(arm))
                //////////// LL SAMPLE 1 ////////////////
                .afterTime(0,CrazySampleSeq.LLSample1DetectPos(arm,slider))
                .afterTime(0.4, limelight_thread::start)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(sampleLL1pick,Math.toRadians(0)),Math.toRadians(0),null,baseAccelConstraint2)
                .waitSeconds(0.05)
                .build();

        Action drop_n_return = drive.actionBuilder(new Pose2d(sampleLL1pick,Math.toRadians(0)))
                .afterTime(0,CrazySampleSeq.LLSample1DropPos(arm,slider,limeUtils.ext_target))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(sampleLL1drop,Math.toRadians(45)),Math.toRadians(-120))
                .waitSeconds(0.3)
                .stopAndAdd(CrazySampleSeq.GripperOpen(arm))
                .afterTime(0,CrazySampleSeq.LLSample1DetectPos(arm,slider))
                .afterTime(0.4, limelight_thread::start)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(sampleLL1pick,Math.toRadians(0)),Math.toRadians(0),null,baseAccelConstraint3)
                .waitSeconds(0.05)
                .build();

        Action drop_n_park = drive.actionBuilder(new Pose2d(sampleLL1pick,Math.toRadians(0)))
                .afterTime(0,CrazySampleSeq.LLSample1DropPos(arm,slider,limeUtils.ext_target))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(sampleLL1drop,Math.toRadians(45)),Math.toRadians(-120))
                .waitSeconds(0.3)
                .stopAndAdd(CrazySampleSeq.GripperOpen(arm))
                .afterTime(0,CrazySampleSeq.TeleOpInit(arm,slider))
                .waitSeconds(0.4)
                .build();


        if(opModeInInit()) {
            Actions.runBlocking(CrazySampleSeq.AutoSampleInit(arm, slider));
        }

        waitForStart();
        limeUtils.limelight3A.start();
        opMode_loop : while (opModeIsActive()){
            switch (autoState){
                case ONE_PLUS_3:
                    Actions.runBlocking(one_plus_three);
//                    Actions.runBlocking(new SequentialAction(
//                            CrazySampleSeq.LLSample1DetectPos(arm,slider),
//                            new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
//                            new SleepAction(1),
//                            new InstantAction(limelight_thread::start),
//                            new SleepAction(0.4)
//                    ));
                    autoState = AutoState.LL_PICK1;
                    break;

                case LL_PICK1:
                    synchronized (this) {
                        generate_pick_trajectory(arm, slider);
                        Actions.runBlocking(lime_pick);
                    }
                        autoState = AutoState.DROP_N_RETURN;
                    Actions.runBlocking(new InstantAction(limelight_thread::interrupt));
                        break;

                case DROP_N_RETURN:
                    Actions.runBlocking(drop_n_return);
                    autoState = AutoState.LL_PICK2;
                    break;

                case LL_PICK2:
                    synchronized (this) {
                        generate_pick_trajectory(arm, slider);
                        Actions.runBlocking(lime_pick);
                    }
                    autoState = AutoState.DROP_N_PARK;
                    Actions.runBlocking(new InstantAction(limelight_thread::interrupt));
                    break;

                case DROP_N_PARK:
                    Actions.runBlocking(drop_n_park);
                    break opMode_loop;
            }
        }

    }
    private void update_target_sample() {
        try {
            if (!limeUtils.yellow_and_blue_samples.isEmpty()) {
                limeUtils.set_target(limeUtils.yellow_and_blue_samples.get(0).field_pos);
                limeUtils.wristState = limeUtils.yellow_and_blue_samples.get(0).orientation? Arm.WristState.WRIST90: Arm.WristState.WRIST0;
            }

        } catch (Exception e) {
            telemetry.addData("No samples to add ", e);
        }
    }

    private void generate_pick_trajectory(Arm arm, Slider slider){
        lime_pick = drive.actionBuilder(new Pose2d(sampleLL1pick,Math.toRadians(0)))
                .stopAndAdd(CrazySampleSeq.LLSample1PickPos(arm,slider,limeUtils.ext_target,limeUtils.yaw_target,limeUtils.wristState))
                .build();
    }
}
