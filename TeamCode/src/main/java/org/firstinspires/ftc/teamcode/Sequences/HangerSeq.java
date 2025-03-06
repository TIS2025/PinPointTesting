package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@Deprecated
public class HangerSeq {
    public static Action HangPart1(Arm arm, Slider slider, Hanger hanger){
        return new SequentialAction(
                new InstantAction(()-> hanger.updateHangerState(Hanger.HangerState.PHASE1_UP)),
                new SleepAction(2),
                new InstantAction(()-> hanger.updateHangerState(Hanger.HangerState.PHASE1_DOWN))
        );
    }
}
