package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class CrazySpecimenSeq {
    public static Action AutoSpecimenInit(Arm arm, Slider slider){
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

    public static Action SpecimenFrontDropPos(Arm arm, Slider slider){
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
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_FRONT_DROP)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST0)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.HOME))
        );
    }

    public static Action Specimen1PickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST135))
        );
    }

    public static Action Specimen1Pick_Drop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.1),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new SleepAction(0.7),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action Specimen2PickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST135))
        );
    }

    public static Action Specimen2Pick_Drop(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.1),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new SleepAction(0.7),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN))
        );
    }

    public static Action Specimen3PickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.WRIST135)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.HORIZONTAL_MAX))
        );
    }

    public static Action Specimen3Pick_Drop(Arm arm, Slider slider){
        /////////// TODO New Pre Pick Pos States
        return new SequentialAction(
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new SleepAction(0.1),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.setTurret(560)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(() -> arm.setElbow(0.3)),
                new InstantAction(() -> arm.setShoulder(0.27)),
                new InstantAction(()-> arm.setYaw(0.56)),
                new InstantAction(()-> arm.setWrist(0.06))
        );
    }

    public static Action Specimen1Scoring(Arm arm, Slider slider){
        return new SequentialAction(
                ////////////////////////////// PICKING ////////////////////////////
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
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new SleepAction(1.8),
                ////////////////////////////// SCORING ////////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1),
                ///////////////////////////// PICK_POS ///////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action Specimen2Scoring(Arm arm, Slider slider){
        return new SequentialAction(
                ////////////////////////////// PICKING ////////////////////////////
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
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new SleepAction(1.8),
                ////////////////////////////// SCORING ////////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1),
                ///////////////////////////// PICK_POS ///////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK))

        );
    }

    public static Action Specimen3Scoring(Arm arm, Slider slider){
        return new SequentialAction(
                ////////////////////////////// PICKING ////////////////////////////
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
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new SleepAction(1.8),
                ////////////////////////////// SCORING ////////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1),
                ///////////////////////////// PICK_POS ///////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action Specimen4Scoring(Arm arm, Slider slider){
        return new SequentialAction(
                ////////////////////////////// PICKING ////////////////////////////
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
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new SleepAction(1.8),
                ////////////////////////////// SCORING ////////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.25),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.1),
                ///////////////////////////// PICK_POS ///////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.2),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK))
        );
    }

    public static Action Specimen5Scoring(Arm arm, Slider slider){
        return new SequentialAction(
                ////////////////////////////// PICKING ////////////////////////////
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
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new SleepAction(2.5),
                ////////////////////////////// SCORING ////////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_DROP)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                ///////////////////////////// PICK_POS ///////////////////////////
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateWristState(Arm.WristState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.3),
                new InstantAction(()-> slider.updateTurretState(Slider.TurretState.SPECIMEN_PRE_PICK)),
                new SleepAction(0.3),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.SPECIMEN_PRE_PICK)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.SPECIMEN_PRE_PICK))
        );
    }
}
