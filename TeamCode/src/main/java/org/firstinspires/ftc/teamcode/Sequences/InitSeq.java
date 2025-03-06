package org.firstinspires.ftc.teamcode.Sequences;

import static org.firstinspires.ftc.teamcode.Globals.MotorConst.extTimeConst;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class InitSeq {
    public InitSeq(Arm arm, Hanger hanger, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.HOME)),
//                        new InstantAction(()->hanger.updateHangerState(Hanger.HangerState.INIT)),
                        new SleepAction(extTime),
                        new InstantAction(()->slider.updateTurretState(Slider.TurretState.INIT)),
                        new SleepAction(0.4),
                        new InstantAction(()->arm.updateElbowState(Arm.ElbowState.INIT)),
                        new InstantAction(()->arm.updateGripperState(Arm.GripperState.INIT)),
                        new InstantAction(()->arm.updateYawState(Arm.YawState.NEUTRAL)),
                        new InstantAction(()->arm.updateWristState(Arm.WristState.INIT)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.INIT))
                )
        );
    }
}
