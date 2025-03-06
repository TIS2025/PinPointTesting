package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class CrazySampleSeq {
    public static Action AutoSampleInit(Arm arm, Slider slider){
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

    public static Action PreloadDropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(1.4),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP)),
                new InstantAction(()-> arm.setYaw(0.35))
        );
    }

    public static Action GripperOpen(Arm arm){
        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.OPEN)),
                new SleepAction(0.2)
        );
    }

    public static Action GripperClose(Arm arm){
        return new SequentialAction(
                new InstantAction(()-> arm.updateGripperState(Arm.GripperState.CLOSE)),
                new SleepAction(0.25)
        );
    }

    public static Action Sample1PickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(0.2),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.setWrist(0)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new SleepAction(0.8),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.INTAKE))
        );
    }

    public static Action Sample1DropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(1.3),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP)),
                new InstantAction(()-> arm.setYaw(0.3)),
                new SleepAction(0.1)
        );
    }

    public static Action Sample2PickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(0.2),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> arm.setYaw(0.365)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(()->arm.setWrist(0.18)),
                new SleepAction(0.8),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.INTAKE))
        );
    }

    public static Action Sample2DropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(1.4),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP)),
                new InstantAction(()-> arm.setYaw(0.4)),
                new SleepAction(0.1)
        );
    }

    public static Action Sample3PickPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(0.2),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.AUTO_PRE_INTAKE)),
                new InstantAction(()->arm.setWrist(0.41)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.setYaw(0.175)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.INTAKE))
        );
    }

    public static Action Sample3DropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.5),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(1.3),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP)),
                new InstantAction(()-> arm.setYaw(0.4)),
                new SleepAction(0.1)
        );
    }

    public static Action LLSample1DetectPos(Arm arm, Slider slider){
        // TODO States
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(0.2),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.setExtInch(6))
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
                new SleepAction(0.3)
        );
    }

    public static Action LLSample1DropPos(Arm arm, Slider slider,double ext){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new SleepAction(0.5),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(1.6),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP)),
                new SleepAction(0.1)
        );
    }

    public static Action LLSample2DetectPos(Arm arm, Slider slider){
        // TODO States
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new SleepAction(0.2),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(0.8),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.DOWN)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.HOME)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.HOME)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.setExt(300)),
                new SleepAction(0.4),
                new InstantAction(()-> arm.setYawDegrees(70))
        );
    }

    public static Action LLSample2PickPos(Arm arm, Slider slider, double ext, double angle, Arm.WristState wristState){
        return  new SequentialAction(
                new InstantAction(()-> slider.setExtInch(ext)),
                new InstantAction(() -> arm.setWristDegrees(wristState,angle)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.setYawDegrees(angle)),
                new InstantAction(() -> arm.updateShoulderState(Arm.ShoulderState.INTAKE)),
                new InstantAction(() -> arm.updateElbowState(Arm.ElbowState.INTAKE))
        );
    }

    public static Action LLSample2DropPos(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.POST_INTAKE)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.POST_INTAKE)),
                new InstantAction(()->slider.updateExtState(Slider.ExtState.MIN)),
                new SleepAction(1),
                new InstantAction(()->slider.updateTurretState(Slider.TurretState.UP)),
                new SleepAction(0.4),
                new InstantAction(()-> slider.updateExtState(Slider.ExtState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.PRE_BUCKET_DROP)),
                new InstantAction(()-> arm.updateYawState(Arm.YawState.NEUTRAL)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST90)),
                new SleepAction(1),
                new InstantAction(()-> arm.updateElbowState(Arm.ElbowState.BUCKET_DROP)),
                new InstantAction(()-> arm.updateShoulderState(Arm.ShoulderState.BUCKET_DROP))
        );
    }

    public static Action TeleOpInit(Arm arm, Slider slider){
        return new SequentialAction(
                new InstantAction(()->slider.updateExtState(Slider.ExtState.INIT)),
                new InstantAction(()->arm.updateElbowState(Arm.ElbowState.AUTO_INIT)),
                new InstantAction(()->arm.updateShoulderState(Arm.ShoulderState.AUTO_INIT)),
                new InstantAction(()->arm.updateYawState(Arm.YawState.AUTO_INIT)),
                new InstantAction(()->arm.updateGripperState(Arm.GripperState.OPEN)),
                new InstantAction(()->arm.updateWristState(Arm.WristState.WRIST0))
        );
    }
}
