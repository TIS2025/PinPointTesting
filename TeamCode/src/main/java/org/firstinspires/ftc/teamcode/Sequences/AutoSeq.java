package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class AutoSeq {
    public static Action SampleInit(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.AUTO_INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_INIT)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(1),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                new SleepAction(1),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.CLOSE))
        );
    }

    public static Action SpecimenInit(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_SPECIMEN_INIT)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new SleepAction(2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE))
        );
    }

    public static Action SpecimenDropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PICK)),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_DROP)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_DROP))
        );
    }

    public static Action SpecimenPickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action SpecimenPick(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PICK)),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_DROP)),
                new SleepAction(0.15),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_DROP))
        );
    }

    public static Action SpecimenDrop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.15)
        );
    }

    public static Action SamplePickPosFromSpecimen(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.DOWN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.FIRST_SPECIMEN_PICK)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL))
        );
    }

    public static Action SamplePickPosFromSpecimen90(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.DOWN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.FIRST_SPECIMEN_PICK)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.AUTO_PICK_LEFT2)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL))
        );
    }

    public static Action SamplePickPosFromSpecimenYawLeft(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.DOWN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.AUTO_PICK_LEFT2)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.5),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.AUTO_PICK_LEFT2))
        );
    }

    public static Action SamplePickPosFromObs(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.FIRST_SPECIMEN_PICK)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL))
        );
    }

    public static Action SamplePickForSpecimen(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.35),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0))
        );
    }

    public static Action SamplePickForSpecimen90(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.35),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.AUTO_PICK_LEFT2))
        );
    }

    public static Action SampleDropObsZone(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(0.35),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1)
        );
    }

    public static Action SpecimenPickPosFromSampleDrop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.25),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK))

        );
    }

    public static Action SampleDropYawRight(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_RIGHT)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))
        );
    }

    public static Action SampleDropYawLeft(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_SAMPLE_DROP_LEFT)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP))
        );
    }

    public static Action SampleDropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP))
        );
    }
    public static Action SamplePickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new InstantAction(() -> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(() -> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(() -> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action SamplePickYawRight(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_PICK_RIGHT)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.AUTO_PICK_RIGHT)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.5),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP))

        );
    }

    public static Action SamplePickYawLeft(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateYawState(Arm.YawState.AUTO_PICK_LEFT)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.AUTO_PICK_LEFT)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP))

        );
    }

    public static Action SamplePick(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.2),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.UP))

        );
    }

    public static Action HomePos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.PRE_INTAKE)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN))
        );
    }

    public static Action TeleOpInit(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.AUTO_INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_INIT)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0))
        );
    }
}
