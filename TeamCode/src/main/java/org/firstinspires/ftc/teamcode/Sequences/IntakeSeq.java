package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@Deprecated
public class IntakeSeq {
    public static Action Home(Arm arm, Slider slider) {
        return new ParallelAction(
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.HOME)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action PreSampleIntake(Arm arm) {
        return new ParallelAction(
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SampleIntake(Arm arm) {
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.CLOSE))
        );
    }

    public static Action PostSampleIntake(Arm arm) {
        return new ParallelAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE))
        );
    }
}