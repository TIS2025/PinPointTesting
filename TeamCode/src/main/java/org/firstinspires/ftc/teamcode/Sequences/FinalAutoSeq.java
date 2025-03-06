package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@Config
public class FinalAutoSeq {
    public static double turretTimeConst = 0.55;
    public static double extTimeConst = 1.3;

    public static Action Init(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.AUTO_INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_INIT)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(2),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.CLOSE))
        );
    }

    public static Action SamplePickPosNoExtNoYaw(Arm arm, Slider slider, int ExtPos){

        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
//                new SleepAction(extTime),
                new SleepAction(0.9),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new SleepAction(0.5),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0))
        );
    }

    public static Action SamplePickPosNoExtLeftYaw(Arm arm, Slider slider, int ExtPos){

        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new SleepAction(0.1),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.AUTO_PICK_LEFT)),
                new SleepAction(0.3),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_PICK_LEFT))
        );
    }

    public static Action SamplePickPosFullExtRightYaw(Arm arm, Slider slider, int ExtPos, int TurPos){

        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;
        double turTime = (double) Math.abs(TurPos - MotorConst.turretDown)/MotorConst.turretDown * turretTimeConst;

        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                        new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                        new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                        new InstantAction(()->arm.updateYawState(Arm.YawState.NEUTRAL)),
                        new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                        new InstantAction(()->arm.updateWristState(Arm.WristState.AUTO_PICK_RIGHT))
                ),
                new SleepAction(extTime),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_PICK_RIGHT)),
                new SleepAction(turTime),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX))
        );
    }

    public static Action SampleDropPosYawLeft(Arm arm, Slider slider, int ExtPos, int TurPos){

        double turTime = (double) Math.abs(TurPos - MotorConst.turretUp)/MotorConst.turretDown * turretTimeConst;
        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(turTime),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_LEFT))
        );
    }

    public static Action SampleDropPosYawRight(Arm arm, Slider slider, int ExtPos, int TurPos){

        double turTime = (double) Math.abs(TurPos - MotorConst.turretUp)/MotorConst.turretDown * turretTimeConst;
        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(turTime),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_RIGHT))
        );
    }

    public static Action SampleDropPosYawNeutral(Arm arm, Slider slider, int ExtPos, int TurPos){

        double turTime = (double) Math.abs(TurPos - MotorConst.turretUp)/MotorConst.turretDown * turretTimeConst;
        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(turTime),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL))
        );
    }

    public static Action SpecimenFrontDropPos(Arm arm, Slider slider){

//        double turTime = (double) Math.abs(TurPos - MotorConst.turretUp)/MotorConst.turretDown * turretTimeConst;
//        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.SPECIMEN_FRONT_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_FRONT_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_FRONT_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_FRONT_DROP)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_FRONT_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_FRONT_DROP))
        );
    }

    public static Action SpecimenFrontDrop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_FRONT_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.INIT)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.INIT))
        );
    }

    public static Action SamplePick(Arm arm)
    {
        return new SequentialAction(
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0))
        );
    }

    public static Action SampleDrop(Arm arm)
    {
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL))
        );
    }

    public static Action SampleSideDrop(Arm arm){
        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))
        );
    }

    public static Action TeleOpInit(Arm arm, Slider slider, int ExtPos){

        double extTime = (double) Math.abs(ExtPos - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.AUTO_INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_INIT)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(extTime),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP))
        );
    }
}
