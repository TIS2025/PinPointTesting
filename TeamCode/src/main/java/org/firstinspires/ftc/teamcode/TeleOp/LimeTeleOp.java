package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.FinalSeq;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.VisionUtils.LimeLightUtils;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(group = "LL", name = "TeleOp Limelight")
public class LimeTeleOp extends LinearOpMode {
    static List<Action> ftc = new ArrayList<>();
    MecanumDrive drive;
    static double turn_coeff = 0.5;
    double botHeading;

    public static double inch_target;
    public static double angle_target;
    public static Arm.WristState wristState;

    LimeLightUtils limeUtils;

    enum BotState{
        SAMPLE_MODE,
        SPECIMEN_MODE
    }

    enum SampleState{
        HOME_POS,
        PICK_POS,
        PICK,
        DROP_POS,
    }

    enum SpecimenState{
        PICK_POS,
        DROP_POS,
    }

    enum DriveState{
        FIELD_CENTRIC,
        BOT_CENTRIC
    }

    SampleState sampleState = SampleState.HOME_POS;
    SpecimenState specimenState = SpecimenState.PICK_POS;
    BotState intakeState = BotState.SAMPLE_MODE;
    DriveState driveState = DriveState.BOT_CENTRIC;

    //FLAGS
    boolean HANGER_FLAG1 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);

        drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);
        limeUtils = new LimeLightUtils(hardwareMap);
        limeUtils.limelight3A.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(100);

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

        int slider_pos = 0;

        boolean wrist_rotate = true;
        double drive_coeff = 1;

        new InitSeq(arm,hanger,slider);

        while (opModeInInit()){
            P1.copy(C1);
            C1.copy(gamepad1);
            P2.copy(C2);
            C2.copy(gamepad2);

            if(C1.left_bumper){
                slider.setExt(robot.extLeft.getCurrentPosition() - 25);
            }
            if(C1.right_bumper){
                slider.setExt(robot.extLeft.getCurrentPosition() + 25);
            }

            if(C1.left_trigger>0.5)
            {
                slider.setTurret(robot.turret.getCurrentPosition() - 25);
            }
            if(C1.right_trigger>0.5){
                slider.setTurret(robot.turret.getCurrentPosition() + 25);
            }

            if(C2.left_stick_button && !P2.left_stick_button && driveState== DriveState.BOT_CENTRIC){
                driveState = DriveState.FIELD_CENTRIC;
            }
            if(C2.left_stick_button && !P2.left_stick_button && driveState== DriveState.FIELD_CENTRIC){
                driveState = DriveState.BOT_CENTRIC;
            }

            telemetry.addData("Slider Pos",robot.extLeft.getCurrentPosition());
            telemetry.addData("Turret Pos",robot.turret.getCurrentPosition());
            telemetry.update();
        }

//        robot.reset_encoders();
        waitForStart();
        limeUtils.limelight3A.start();
        while(opModeIsActive()){
            P1.copy(C1);
            C1.copy(gamepad1);
            P2.copy(C2);
            C2.copy(gamepad2);

            limeUtils.clear_samples();
            limeUtils.filter(limeUtils.limelight3A.getLatestResult(),telemetry);

            drive.setDrivePowers(
                    driveCommand(C1,drive_coeff)
            );

            botHeading = drive.pose.heading.toDouble();
            if (C1.options && !P1.options && driveState == DriveState.FIELD_CENTRIC) {
                drive.navxMicro.initialize();
            }

            ftc = updateAction();

            if(C1.left_trigger>0.5 && !HANGER_FLAG1) {
                drive_coeff = 0.25;
                turn_coeff = 0.25;
            }
            else if(C1.left_trigger<0.5 && !HANGER_FLAG1) {
                drive_coeff = 1;
                turn_coeff = 0.6;
            }
            else if(C1.left_trigger>0.5 && HANGER_FLAG1) {
                drive_coeff = 0.25;
                turn_coeff = 0.25;
            }

            if(C2.dpad_down){
                HANGER_FLAG1 = true;
            }

            if(C2.dpad_left && !P2.dpad_left && specimenState == SpecimenState.PICK_POS){
                intakeState = BotState.SAMPLE_MODE;
                sampleState = SampleState.HOME_POS;
                ftc.add(FinalSeq.HomePos(arm,slider));
            }

            if(C2.dpad_right && !P2.dpad_right && sampleState == SampleState.HOME_POS){
                specimenState = SpecimenState.PICK_POS;
                intakeState = BotState.SPECIMEN_MODE;
                ftc.add(FinalSeq.SpecimenPickPos(arm,slider));
            }

            if(C1.a && !P1.a && intakeState == BotState.SAMPLE_MODE && (sampleState == SampleState.HOME_POS || sampleState == SampleState.PICK_POS || sampleState == SampleState.PICK)){
                ftc.add(FinalSeq.LLSample1DetectPos(arm,slider));
                sampleState = SampleState.HOME_POS;
            }

            if(C1.b && !P1.b && intakeState == BotState.SAMPLE_MODE && (sampleState == SampleState.HOME_POS || sampleState == SampleState.PICK)){
                update_target_pos();
                ftc.add(FinalSeq.LLSample1PickPos(arm, slider, limeUtils.ext_target, limeUtils.yaw_target, limeUtils.wristState));
                sampleState = SampleState.PICK_POS;
            }

            if(C1.x && !P1.x && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                ftc.add(FinalSeq.SamplePick(arm,slider));
                sampleState = SampleState.PICK;
                wrist_rotate = true;
            }

            if(C1.left_bumper && !P1.left_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK){
                ftc.add(FinalSeq.SampleDropPos(arm,slider));
                sampleState = SampleState.DROP_POS;
            }

            if(C1.right_bumper && !P1.right_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.DROP_POS){
                ftc.add(FinalSeq.SampleDrop(arm,slider));
                sampleState = SampleState.HOME_POS;
            }

            if(C1.right_bumper && !P1.right_bumper && intakeState == BotState.SPECIMEN_MODE && specimenState == SpecimenState.DROP_POS){
                ftc.add(FinalSeq.SpecimenDrop(arm,slider));
                specimenState = SpecimenState.PICK_POS;
            }
            if(C1.left_bumper && !P1.left_bumper && intakeState == BotState.SPECIMEN_MODE && specimenState == SpecimenState.PICK_POS) {
                ftc.add(FinalSeq.SpecimenPick(arm,slider));
                specimenState = SpecimenState.DROP_POS;
            }

            if(C2.left_bumper && !P2.left_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS)
            {
                slider_pos = 1200;
                slider.setExt(slider_pos);
            }

            if(C2.right_bumper && !P2.right_bumper && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                slider_pos=0;
                slider.setExt(slider_pos);
            }

            if(C1.y && !P1.y && intakeState == BotState.SAMPLE_MODE && sampleState == SampleState.PICK_POS){
                wrist_rotate = !wrist_rotate;
//                arm.updateWristState(wrist_rotate? Arm.WristState.WRIST0: Arm.WristState.WRIST90);
                robot.wrist.setPosition(wrist_rotate? ServoConst.wrist0 : ServoConst.wrist90);
            }


            if(C1.left_trigger>0.75 && !(P1.left_trigger>0.75) && HANGER_FLAG1) ftc.add(FinalSeq.PreHang(arm, slider, hanger));
            if(C1.right_trigger>0.75 && !(P1.right_trigger>0.75) && HANGER_FLAG1) ftc.add(FinalSeq.Hang(slider,hanger));

//            telemetry.addData("Turret Pos",robot.turret.getCurrentPosition());
//            telemetry.addData("Ext left Pos",robot.extLeft.getCurrentPosition());
//            telemetry.addData("Ext right Pos",robot.extRight.getCurrentPosition());
//            telemetry.addData("Hanger Pos",robot.hanger.getCurrentPosition());
//            telemetry.addData("Drive State",driveState);
//            telemetry.addData("Bot State",botState);
//            telemetry.addData("Wrist state",wrist_rotate);
//            telemetry.addData("Gripper State",arm.gripperState);
//            telemetry.addData("Drive Current FL",drive.leftFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Drive Current FR",drive.rightFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Drive Current BL",drive.leftBack.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Drive Current BR",drive.rightBack.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Lifter Current L",robot.extLeft.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Lifter Current R",robot.extRight.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Turret Current",robot.turret.getCurrent(CurrentUnit.AMPS));
            limeUtils.print_samples(telemetry);
            telemetry.update();
        }
    }

    private void update_target_pos() {
        try{
        if(!limeUtils.yellow_samples.isEmpty()){
            limeUtils.set_target(limeUtils.yellow_samples.get(0).field_pos);
            limeUtils.wristState = limeUtils.yellow_samples.get(0).orientation? Arm.WristState.WRIST90: Arm.WristState.WRIST0;
        }
        } catch (Exception e) {
            telemetry.addData("Exception",e);
        }
    }

    private static List<Action> updateAction() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : ftc) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
//        runningActions.removeAll(RemovableActions);
        return newActions;
    }

    private PoseVelocity2d driveCommand(Gamepad G,double drive_coeff){
        double x = Math.pow(Range.clip(-G.left_stick_y,-1,1),3)*drive_coeff;
        double y = Math.pow(Range.clip(-G.left_stick_x,-1,1),3)*drive_coeff;
        double angVel = Math.pow(Range.clip(-G.right_stick_x,-1,1),3)*turn_coeff;

        return new PoseVelocity2d(new Vector2d(x,y),angVel);
    }
}
