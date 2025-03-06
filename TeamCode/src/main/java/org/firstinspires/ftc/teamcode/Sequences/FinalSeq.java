package org.firstinspires.ftc.teamcode.Sequences;

import static org.firstinspires.ftc.teamcode.Globals.MotorConst.extTimeConst;
import static org.firstinspires.ftc.teamcode.Globals.MotorConst.turretTimeConst;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class FinalSeq {



    public static Action HomePos(Arm arm, Slider slider) {

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;
        return new SequentialAction(
                new InstantAction(() -> slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.HOME)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.HOME)),
                new SleepAction(extTime+0.2),
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SamplePickPos(Arm arm, Slider slider){

        return new SequentialAction(
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(0.2),
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE))
        );
    }

    public static Action SamplePick(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.1),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN))
        );
    }

    public static Action SampleDropPos(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;
        double turTime = (double) Math.abs(slider.TurretPos() - MotorConst.turretUp)/MotorConst.turretDown * turretTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(turTime),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(extTime),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))

        );
    }


    public static Action SampleDrop(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1.3),
                new InstantAction(() -> slider.updateTurretState(Slider.TurretState.DOWN)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SampleDropObsZone(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extHorizontalMax) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(extTime),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1)
        );
    }

    public static Action SpecimenPickPos(Arm arm, Slider slider){

        double extTime = (double) Math.abs(slider.ExtPos() - MotorConst.extSpecimenPrePick) /MotorConst.extMax * extTimeConst;

        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(extTime + 0.1),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action SpecimenPick(Arm arm, Slider slider){

        double turTime = (double) Math.abs(slider.TurretPos() - MotorConst.turretSpecimenPreDrop)/MotorConst.turretDown * turretTimeConst;

        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PICK)),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_DROP)),
                new SleepAction(0.15 ),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action SpecimenDrop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.15),
                SpecimenPickPos(arm,slider)
        );
    }

    public static Action PreHang(Arm arm, Slider slider, Hanger hanger){
        double extTime = Math.abs((double) (slider.ExtPos() - MotorConst.extMin) /MotorConst.extMax)*1;
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(extTime-0.2),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_HANG)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.PRE_HANG)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.PRE_HANG)),
//                new InstantAction(()->hanger.updateHangerState(Hanger.HangerState.INIT)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.PRE_HANG)),
                new SleepAction(0.2),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.PRE_HANG))
        );
    }

    public static Action Hang(Slider slider, Hanger hanger){
        return new SequentialAction(
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.HANG)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HANG))
        );
    }

    public static Action LimePick(Slider slider, Arm arm, double ext, double angle, Arm.WristState wristState){
        return  new SequentialAction(
                new InstantAction(()-> slider.setExtInch(ext)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(() -> arm.setWristDegrees(wristState,angle)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.setYawDegrees(angle)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE))
        );
    }

    public static Action LLSample1PickPos(Arm arm, Slider slider, double ext, double angle, Arm.WristState wristState){
        return  new SequentialAction(
                new ParallelAction(
                        new InstantAction(()-> slider.setExtInch(ext)),
                        new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                        new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                        new InstantAction(() -> arm.setYawDegrees(angle)),
                        new InstantAction(() -> arm.setWristDegrees(wristState,angle))
                ),
                new SleepAction(0.05*(Math.abs(ext-6))),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN))
        );
    }

    public static Action LLSample1DetectPos(Arm arm, Slider slider){
        // TODO States
        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new SleepAction(0.5),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.setExtInch(6))
        );
    }
}
