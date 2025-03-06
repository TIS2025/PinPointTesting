package org.firstinspires.ftc.teamcode.Auto;//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Sequences.KSleep;
//import org.firstinspires.ftc.teamcode.VisionUtils.LimeLightUtils;
//import org.firstinspires.ftc.teamcode.hardware.Globals;
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.instantcommands.BucketCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.BucketFlapCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.ClutchCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.ElbowCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.ElevatorCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.IntakeGripperCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.ShoulderCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.SpecimenArmCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.SpecimenGripCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.ViperCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.WristCommand;
//import org.firstinspires.ftc.teamcode.instantcommands.XExtensionCommand;
//import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SpecimenSubsystem;
//import org.opencv.core.Point;
//
//import java.util.Arrays;
//
//@Autonomous(group = "SAMPLE AUTO", name = "🔵 Sample 1+6 After Scrimage ")
//public class New_LL_Auto extends LinearOpMode {
//    // TODO REPLACE ELBOW STATE FROM GRIP MODE TO SAMPLE PICK
//    private final RobotHardware robot = RobotHardware.getInstance();
//    //Subsystems
//    IntakeSubsystem intake = null;
//    SpecimenSubsystem specimen = null;
//    ElevatorSubsystem elevator = null;
//    //Drive
//    private MecanumDrive drive = null;
//
//    //TODO  LIME-LIGHT STUFF BEGINS =========================================================================================================================
//    private boolean orient = false;
//    private Point position = new Point(0, -1);
//    public LimeLightUtils limeUtil = null;
//    public static double state = 1;
//
//    public Action bucketDrop;
//
//    public Action submersibleTraj;
//
//    public static Action ParkingTraj;
//
//
//    private double increment = 0;
//    static boolean LimePick = false;
//
//    AccelConstraint baseAccelConstraint2 = new ProfileAccelConstraint(-45, 90);
//    AccelConstraint baseAccelConstraint3 = new ProfileAccelConstraint(-45, 100);
//    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
//            new TranslationalVelConstraint(90),
//            new AngularVelConstraint(Math.PI / 2)
//    ));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//
//        intake = new IntakeSubsystem(robot);
//        specimen = new SpecimenSubsystem(robot);
//        elevator = new ElevatorSubsystem(robot);
//        limeUtil = new LimeLightUtils(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-41, -64, Math.toRadians(90));
//        drive = new MecanumDrive(hardwareMap, startPose);
//        Thread limelightThread = new Thread(() -> {
//            while (opModeIsActive()) {
//                limeUtil.clear_samples();
//                limeUtil.filter_color(limeUtil.limelight3A.getLatestResult(), telemetry, "BlueYellow");
//                synchronized (this) {
//                    update_target_sample();
//                    limeUtil.getExtYawWrist(position, orient, telemetry);
//                }
//                limeUtil.print_samples(telemetry);
//                telemetry.update();
//            }
//        });
//
//        //TODO ===============================================TRAJECTORIES =============================================================
//
//        Action trajectoryAction = drive.actionBuilder(startPose)
//                //PRELOAD UP
//                .afterTime(0.1, () -> {
//                            new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);
//                            new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
//                            Actions.runBlocking(new InstantAction(() -> robot.wrist.setPosition(0.52)));
//                            Actions.runBlocking(new InstantAction(() -> robot.elbow.setPosition(0.136)));
//                        }
//                )
//                .afterTime(0.7, () -> new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP_SAFE))
////                .afterTime(1.2, () -> new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE))
//
//                //TODO PRELOAD DROP AND 1st PICK
//                .strafeToLinearHeading(new Vector2d(-58.5, -53), Math.toRadians(68))//new Vector2d(-58.5, -53), Math.toRadians(68.3)
//                .stopAndAdd(() -> {
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE);
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.EXTREME);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//                    Actions.runBlocking(new InstantAction(() -> robot.viper.setPosition(0.6128)));
//                    new KSleep(0.35);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//
//                })
//
//                .strafeToLinearHeading(new Vector2d(-56.2001, -52), Math.toRadians(68))//new Vector2d(-57.5, -52), Math.toRadians(68.3)
//                .stopAndAdd(() -> {
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK);
//                    new KSleep(0.2);
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE);
//                    new KSleep(0.25);
//                    //TRANSFER SEQ
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                    new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new WristCommand(intake, IntakeSubsystem.WristState.VERTICAL);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new KSleep(0.1);
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//                    new KSleep(0.5);//0.4
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN);
//                    new KSleep(0.2);
//
//                    //AFTER TRANSFER TO PRE PICK
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);
////                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE);
//
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
//                })
//
//                //TODO 2nd DROP
//                .strafeToLinearHeading(new Vector2d(-62, -52.5), Math.toRadians(82))//-61, -52.5b    85
//                .stopAndAdd(() -> {
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.EXTREME);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP_SAFE);
//                    new KSleep(0.55);//0.6
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE);
//                    new KSleep(0.3);//0.2
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//
//                    //PICK
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK);
//                    new KSleep(0.2);
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE);
//                    new KSleep(0.25);
//
//
//                    //TRANSFER SEQ
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                    new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new WristCommand(intake, IntakeSubsystem.WristState.VERTICAL);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new KSleep(0.2);//
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//                    new KSleep(0.5);//0.4
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN);
//                    new KSleep(0.2);
//
//
//                    //AFTER TRANSFER TO PRE PICK
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE);
//
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
//
//                })
//
//
//                //TODO DROP 3
//                .strafeToLinearHeading(new Vector2d(-64.5, -51), Math.toRadians(95))
//                .stopAndAdd(() -> {
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.EXTREME);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP_SAFE);
//                    new KSleep(0.2);
//                    Actions.runBlocking(new InstantAction(() -> robot.viper.setPosition(0.7339)));
//                    Actions.runBlocking(new InstantAction(() -> intake.setWristWithViper(0.7, 0.7339)));
//                    new KSleep(0.3);//0.4
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE);
//                    new KSleep(0.3);//0.2
//
//
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//
//
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//
//                    //PICK
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK);
//                    new KSleep(0.25);
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE);
//                    new KSleep(0.2);
//
//
//                    //TRANSFER SEQ
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                    new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new WristCommand(intake, IntakeSubsystem.WristState.VERTICAL);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new KSleep(0.2);
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//                    new KSleep(0.5);//0.4
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN);
//                    new KSleep(0.2);
//
//
//                    //AFTER TRANSFER TO PRE PICK
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);
//                    Actions.runBlocking(new InstantAction(() -> robot.elbow.setPosition(0.12)));
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE);
//
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
////                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP);
//                    new KSleep(0.6);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP_SAFE);
//                    new KSleep(0.1);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE);
//                    new KSleep(0.3);
//                })
//
//                .afterTime(0.25, () -> {
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//                })
//                .afterTime(1, () -> {
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                })
//                .setReversed(true)
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-22, -10, Math.toRadians(0)), Math.toRadians(0))//10
//                .build();
//
//
//        Pose2d buckDropPose2 = new Pose2d(-22.001, -10.01, Math.toRadians(0));
//
//        Action bucketDrop2 = drive.actionBuilder(buckDropPose2)
//                .afterTime(0, () -> {
//                    new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_DROP);
//                    new SleepAction(0.1);
//
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//
//                })
//
//                .afterTime(0.4, () -> {
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN);
//                })
//
//                .afterTime(0.8, () -> {
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE);
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);//INIT
//                })
//
//                .afterTime(1, () -> {
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-54.001, -57, Math.toRadians(45)), Math.toRadians(-120), null, baseAccelConstraint2)//-53.2001, -55
//                .stopAndAdd(() -> new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE))
//                .waitSeconds(0.3)//0.3CB
//                .afterTime(0.1, () -> {
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//
//                })
//                .build();
//
//        Pose2d submersibleTrajPose2 = new Pose2d(-54.001, -57, Math.toRadians(45));//-52.2012, -54.1,
//
//        Action submersibleTraj2 = drive.actionBuilder(submersibleTrajPose2)
//                .setReversed(false)
//                .afterTime(0.1, () -> {
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.INIT);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PREPICK);
//                })
////                .setReversed(true)
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-22, -12, Math.toRadians(0)), Math.toRadians(0), null, baseAccelConstraint2)//10
//                .build();
//
//
//        Pose2d buckDropPose3 = new Pose2d(-22.001, -12.01, Math.toRadians(0));
//
//        Action bucketDrop3 = drive.actionBuilder(buckDropPose3)
//                .afterTime(0, () -> {
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_DROP);
//                    new SleepAction(0.1);
//                    new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//
//                })
//                .afterTime(0.4, () -> {
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN);
//                })
//
//                .afterTime(0.8, () -> {
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE);
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);//INIT
//                })
//
//                .afterTime(1, () -> {
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-54.1, -57, Math.toRadians(45)), Math.toRadians(-120), null, baseAccelConstraint2)
//                .stopAndAdd(() -> new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE))
//                .waitSeconds(0.3)//0.3
//                .afterTime(0.1, () -> {
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//
//                })
//                .build();
//        Pose2d submersibleTrajPose3 = new Pose2d(-54.001, -57, Math.toRadians(45));
//
//        Action submersibleTraj3 = drive.actionBuilder(submersibleTrajPose3)
//                .setReversed(false)
//                .afterTime(0.1, () -> {
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.INIT);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PREPICK);
//                })
//                .setReversed(true)
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-22, -8, Math.toRadians(0)), Math.toRadians(0), null, baseAccelConstraint2)//10
//                .build();
//
//
//        Pose2d buckDropPose4 = new Pose2d(-23.0001, -8.001, Math.toRadians(0));
//
//        Action bucketDrop4 = drive.actionBuilder(buckDropPose4)
//                .afterTime(0, () -> {
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP);
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_DROP);
//                    new SleepAction(0.1);
//                    new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//
//                })
//                .afterTime(0.4, () -> {
//                    new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN);
//                })
//
//                .afterTime(0.8, () -> {
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE);
//                    new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK);
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK);//INIT
//                })
//
//                .afterTime(1, () -> {
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HIGH_BASKET);
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.DROP);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-54.01, -57, Math.toRadians(45)), Math.toRadians(-120), null, baseAccelConstraint2)
//                .stopAndAdd(() -> new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.SAFE))
//                .waitSeconds(0.3)//0.3
//                .afterTime(0.1, () -> {
//                    new BucketCommand(specimen, SpecimenSubsystem.BucketState.INIT);
//                    new ElevatorCommand(elevator, ElevatorSubsystem.ElevatorState.HOME);
//                    new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.OPEN);
//
//                })
//                .build();
//
//        Pose2d parkPose = new Pose2d(-54.0001, -57, Math.toRadians(45));
//
//        Action ParkingTraj2 = drive.actionBuilder(parkPose)
//                .setReversed(false)
//                .afterTime(0.1, () -> {
//                    new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK));
//                    new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PREPICK);
//                })
////                .afterTime(0.6,()->)
//                .splineToLinearHeading(new Pose2d(-24, -14, Math.toRadians(0)), Math.toRadians(0), baseVelConstraint, baseAccelConstraint3)
//                .stopAndAdd(() -> new SpecimenArmCommand(specimen, SpecimenSubsystem.SpecimenArmState.SPECIMEN_DROP))
////                .waitSeconds(0.3)
//                .build();
//
//
//        if (opModeInInit()) {
//            telemetry.addLine("ROBOT INIT MODE");
//            Actions.runBlocking(new SequentialAction(
//                    new InstantAction(() -> new BucketFlapCommand(specimen, SpecimenSubsystem.BuckFlapState.CLOSE)),
//                    new SleepAction(0.6),
//                    new InstantAction(() -> new ClutchCommand(elevator, ElevatorSubsystem.CLutchState.INIT)),
//
//                    new InstantAction(() -> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE)),
//                    new InstantAction(() -> new ViperCommand(intake, IntakeSubsystem.ViperState.INIT)),
//                    new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN)),
//                    new InstantAction(() -> new ElbowCommand(intake, IntakeSubsystem.ElbowState.INIT)),
//                    new InstantAction(() -> new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_PICK)),
//                    new InstantAction(() -> new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME)),
//                    new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP))
//
//            ));
//
//        }
//        telemetry.setMsTransmissionInterval(100);
//        limeUtil.limelight3A.pipelineSwitch(0);
//        state = 1;
//        LimePick = false;
//        waitForStart();
//        limeUtil.limelight3A.start();
//        limelightThread.start();
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            if (state == 1) {
//                Actions.runBlocking(new SequentialAction(
//                        trajectoryAction
//                ));
//                state = 2;
//            }
//
//
//            if (state == 2) {
//                sleep(100);
//                synchronized (this) {
//                    Actions.runBlocking(new SequentialAction(
//                            new InstantAction(() -> intake.extendXExtension(limeUtil.ext_target, 1)),
//                            new SleepAction(0.4),//0.4
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN)),
//                            new InstantAction(() -> intake.setWrist(limeUtil.wrist_target)),
//                            new InstantAction(() -> intake.setShoulder(Globals.shoulderSamplePrePick)),
//                            new InstantAction(() -> intake.setElbow(Globals.elbowSamplePrePick)),
//                            new InstantAction(() -> intake.setViper(limeUtil.viper_target)),
//                            new SleepAction(0.2),
//                            new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK)),
//                            new InstantAction(() -> robot.elbow.setPosition(0.12)),//new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK)
//                            new SleepAction(0.2),//0.125
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE)),
//                            new SleepAction(0.25)
//
//
//                    ));
//                    increment = 3;
//                    state = 3;
//                }
//            }
//
//            if (state == 3) {
//
//                Actions.runBlocking(new SequentialAction(
//                                bucketDrop2,
//                                submersibleTraj2
//                        )
//                );
//                state = 4;
//            }
//            if (state == 4) {
//                sleep(100);
//                synchronized (this) {
//                    Actions.runBlocking(new SequentialAction(
//                            new InstantAction(() -> intake.extendXExtension(limeUtil.ext_target, 1)),
//                            new SleepAction(0.4),//0.4
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN)),
//                            new InstantAction(() -> intake.setWrist(limeUtil.wrist_target)),
//                            new InstantAction(() -> intake.setShoulder(Globals.shoulderSamplePrePick)),
//                            new InstantAction(() -> intake.setElbow(Globals.elbowSamplePrePick)),
//                            new InstantAction(() -> intake.setViper(limeUtil.viper_target)),
//                            new SleepAction(0.2),
//                            new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK)),
//                            new InstantAction(() -> robot.elbow.setPosition(0.12)),//new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK)
//                            new SleepAction(0.2),//0.125
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE)),
//                            new SleepAction(0.25)
//
////                            new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP)),
////                            new InstantAction(() -> new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP)),
////                            new InstantAction(() -> new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_DROP)),
////                            new SleepAction(0.1),
////                            new InstantAction(() -> new ViperCommand(intake, IntakeSubsystem.ViperState.INIT)),
////                            new InstantAction(() -> new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME))
//
//                    ));
//                    increment = 4;
//                    state = 5;
//                }
//            }
//            if (state == 5) {
//                Actions.runBlocking(new SequentialAction(
//                                bucketDrop3,
//                                submersibleTraj3
//                        )
//                );
//                state = 6;
//            }
//            if (state == 6) {
//                sleep(100);
//                synchronized (this) {
//                    Actions.runBlocking(new SequentialAction(
//                            new InstantAction(() -> intake.extendXExtension(limeUtil.ext_target, 1)),
//                            new SleepAction(0.4),//0.4
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN)),
//                            new InstantAction(() -> intake.setWrist(limeUtil.wrist_target)),
//                            new InstantAction(() -> intake.setShoulder(Globals.shoulderSamplePrePick)),
//                            new InstantAction(() -> intake.setElbow(Globals.elbowSamplePrePick)),
//                            new InstantAction(() -> intake.setViper(limeUtil.viper_target)),
//                            new SleepAction(0.2),
//                            new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK)),
//                            new InstantAction(() -> robot.elbow.setPosition(0.12)),//new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK)
//                            new SleepAction(0.2),//0.125
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE)),
//                            new SleepAction(0.25)
//
////                            new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP)),
////                            new InstantAction(() -> new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP)),
////                            new InstantAction(() -> new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_DROP)),
////                            new SleepAction(0.1),
////                            new InstantAction(() -> new ViperCommand(intake, IntakeSubsystem.ViperState.INIT)),
//////                                new SleepAction(0.15),
////                            new InstantAction(() -> new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME))
////
//                    ));
//                    state = 7;
//                }
//            }
//            if (state == 7) {
//                Actions.runBlocking(new SequentialAction(
//                                bucketDrop4,
//                                ParkingTraj2
//                        )
//                );
//                state = 8;
//
//            }
//
//            if (state == 8) {
//                sleep(100);
//                synchronized (this) {
//                    Actions.runBlocking(new SequentialAction(
//                            new InstantAction(() -> intake.extendXExtension(limeUtil.ext_target, 1)),
//                            new SleepAction(0.3),//0.4
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.OPEN)),
//                            new InstantAction(() -> intake.setWrist(limeUtil.wrist_target)),
//                            new InstantAction(() -> intake.setShoulder(Globals.shoulderSamplePrePick)),
//                            new InstantAction(() -> intake.setElbow(Globals.elbowSamplePrePick)),
//                            new InstantAction(() -> intake.setViper(limeUtil.viper_target)),
//                            new SleepAction(0.2),
//                            new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK)),
//                            new InstantAction(() -> robot.elbow.setPosition(0.12)),//new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_PICK)
//                            new SleepAction(0.2),//0.125
//                            new InstantAction(() -> new IntakeGripperCommand(intake, IntakeSubsystem.IntakeGripperState.CLOSE)),
//                            new SleepAction(0.25)
////                            new ParallelAction(
////                                    new InstantAction(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_DROP)),
////                                    new InstantAction(() -> new ElbowCommand(intake, IntakeSubsystem.ElbowState.SAMPLE_DROP)),
////                                    new InstantAction(() -> new WristCommand(intake, IntakeSubsystem.WristState.SAMPLE_DROP)),
//////                            new SleepAction(0.1),
////                                    new InstantAction(() -> new ViperCommand(intake, IntakeSubsystem.ViperState.INIT)),
////                                    new InstantAction(() -> new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME))
////                            )
//                    ));
//                    state = 9;
//                }
//            }
//            if (state == 9) {
//                sleep(200);
//                limelightThread.interrupt();
//                state = 10;
//            }
//
//
//        }
//    }
//
//    private void update_target_sample() {
//        try {
//            if (!limeUtil.relevantSamples.isEmpty()) {
//                position = limeUtil.relevantSamples.get(0).field_pos;
//                orient = limeUtil.relevantSamples.get(0).orientation;
////                pos = orient ? Globals.vertical : Globals.horizontal;
//            }
//
//        } catch (Exception e) {
//            telemetry.addData("No samples to add ", e);
//        }
//    }
//
//}