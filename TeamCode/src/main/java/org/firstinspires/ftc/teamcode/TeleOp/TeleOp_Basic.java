package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.FinalSeq;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Deprecated
@TeleOp(group = "TeleOp", name = "Basic TeleOp")
public class TeleOp_Basic extends LinearOpMode {
    static List<Action> ftc = new ArrayList<>();
    MecanumDrive drive;

    enum BotState{
        SAMPLE_MODE,
        SPECIMEN_MODE
    }

    BotState botState = BotState.SAMPLE_MODE;

    int specimenState = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));
        robot.init_encoders();
        Slider slider = new Slider(robot);
        Hanger hanger = new Hanger(robot);
        Arm arm = new Arm(robot);

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

        int slider_pos = 0;

        boolean wrist_rotate = true;
        double distance;

        while (opModeInInit()){
            P2.copy(C2);
            C2.copy(gamepad2);

            if(C2.left_bumper){
                slider.setExt(robot.extLeft.getCurrentPosition() - 25);
            }
            if(C2.right_bumper){
                slider.setExt(robot.extLeft.getCurrentPosition() + 25);
            }
        }

        robot.reset_encoders();
        new InitSeq(arm,hanger,slider);
        waitForStart();
        while(opModeIsActive()){
            P1.copy(C1);
            C1.copy(gamepad1);
            P2.copy(C2);
            C2.copy(gamepad2);
            distance = robot.colorSensor.getDistance(DistanceUnit.MM);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -C1.left_stick_y,
                                    -C1.left_stick_x
                            ),
                            -C1.right_stick_x*0.5
                    )
            );
            ftc = updateAction();

            //STATE SELECT
            if(C2.dpad_left && !P2.dpad_left){
                botState = BotState.SAMPLE_MODE;
                ftc.add(FinalSeq.HomePos(arm,slider));
            }
            if(C2.dpad_right && !P2.dpad_right){
                botState = BotState.SPECIMEN_MODE;
                ftc.add(FinalSeq.SpecimenPickPos(arm,slider));
            }

            if(C1.a && !P1.a && botState == BotState.SAMPLE_MODE){
//                ftc.add(IntakeSeq.Home(arm,slider));
                ftc.add(FinalSeq.HomePos(arm,slider));
            }
            if(C1.x && !P1.x && botState == BotState.SAMPLE_MODE){
//                ftc.add(IntakeSeq.PreSampleIntake(arm));
                ftc.add(FinalSeq.SamplePickPos(arm,slider));
            }
            if(C1.y && !P1.y && botState == BotState.SAMPLE_MODE){
                ftc.add(FinalSeq.SamplePick(arm,slider));
            }

            if(C1.left_bumper && !P1.left_bumper && botState == BotState.SAMPLE_MODE){
                ftc.add(FinalSeq.SampleDropPos(arm,slider));
            }
            if(C1.b && !P1.b && botState == BotState.SAMPLE_MODE){
                ftc.add(FinalSeq.SampleDrop(arm,slider));
            }
            if(C1.right_trigger>0.75 && !(P1.right_trigger>0.75) && botState == BotState.SPECIMEN_MODE){
                ftc.add(FinalSeq.SpecimenDrop(arm,slider));
            }
            if(C1.left_trigger>0.75 && !(P1.left_trigger>0.75) && botState == BotState.SPECIMEN_MODE) {
                ftc.add(FinalSeq.SpecimenPick(arm,slider));
            }

            if(C2.left_bumper && !P2.left_bumper && botState == BotState.SAMPLE_MODE){
                slider_pos+=400;
                slider_pos = Math.min(slider_pos,1200);
                slider.setExt(slider_pos);
            }
            if(C2.right_bumper && !P2.right_bumper && botState == BotState.SAMPLE_MODE){
                slider_pos-=400;
                slider_pos = Math.max(slider_pos,0);
                slider.setExt(slider_pos);
            }

            if(C2.a && !P2.a && botState == BotState.SAMPLE_MODE){
                wrist_rotate = !wrist_rotate;
                arm.updateWristState(wrist_rotate? Arm.WristState.WRIST0: Arm.WristState.WRIST90);
            }

            if(C2.left_trigger>0.75) hanger.setHanger(1100);
            if(C2.right_trigger>0.75) hanger.setHanger(500);

            telemetry.addData("Turret Pos",robot.turret.getCurrentPosition());
            telemetry.addData("Ext left Pos",robot.extLeft.getCurrentPosition());
            telemetry.addData("Ext right Pos",robot.extRight.getCurrentPosition());
            telemetry.addData("Hanger Pos",robot.hanger.getCurrentPosition());
            telemetry.addData("Shoulder",robot.shoulder.getPosition());
            telemetry.addData("Yaw",robot.yaw.getPosition());
            telemetry.addData("Elbow",robot.elbow.getPosition());
            telemetry.addData("Wrist",robot.wrist.getPosition());
            telemetry.addData("Gripper",robot.gripper.getPosition());
            telemetry.addData("Distance",distance);
            telemetry.addData("intake_state",specimenState);
            telemetry.update();
        }
    }

    private static List<Action> updateAction() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();

        for (Action action : ftc) {
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        return newActions;
    }
}
