package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Slider {
    private final RobotHardware robot;

    public int targetPosition = 0;
    public double maxPower = 0.8;
    public boolean isMotionProfileActive = false;
    public double rampUpDistance;
    public double rampDownDistance;
    public int distanceToMove;

    public Slider(RobotHardware  robot){
        this.robot = robot;
    }

    public enum ExtState{
        MIN,
        MAX,
        HORIZONTAL_MAX,
        INIT,
        BUCKET_DROP,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP,
        PRE_HANG,
        HANG,
        FIRST_SPECIMEN_PICK,
        SPECIMEN_FRONT_DROP,
    }
    public enum TurretState{
        UP,
        DOWN,
        INIT,
        PRE_BUCKET_DROP,
        SPECIMEN_PRE_PICK,
        SPECIMEN_PRE_DROP,
        SPECIMEN_DROP,
        PRE_HANG,
        HANG,
        MOTION_PROFILE1,
        MOTION_PROFILE2,
        SPECIMEN_FRONT_DROP
    }

    public ExtState extState = ExtState.INIT;
    public TurretState turretState = TurretState.INIT;

    public void updateExtState(@NonNull ExtState state){
        switch (state){
            case INIT:
                setExt(MotorConst.extInit);
                break;
            case MIN:
                setExt(MotorConst.extMin);
                break;
            case HORIZONTAL_MAX:
                setExt(MotorConst.extHorizontalMax);
                break;
            case MAX:
                setExt(MotorConst.extMax);
                break;
            case BUCKET_DROP:
                setExt(MotorConst.extHighBucketDrop);
                break;
            case SPECIMEN_PRE_PICK:
                setExt(MotorConst.extSpecimenPrePick);
                break;
            case SPECIMEN_PRE_DROP:
                setExt(MotorConst.extSpecimenPreDrop);
                break;
            case SPECIMEN_DROP:
                setExt(MotorConst.extSpecimenDrop);
                break;
            case PRE_HANG:
                setExt(MotorConst.extPreHang);
                break;
            case HANG:
                setExt(MotorConst.extHang);
                break;
            case FIRST_SPECIMEN_PICK:
                setExt(MotorConst.extFirstSpecimen);
                break;
            case SPECIMEN_FRONT_DROP:
                setExt(MotorConst.extSpecimenFrontDrop);
                break;
        }
        extState = state;
    }

    public void updateTurretState(@NonNull TurretState state){
        switch (state){
            case INIT:
                setTurret(MotorConst.turretInit);
                break;
            case UP:
                setTurret(MotorConst.turretUp);
                break;
            case DOWN:
                setTurret(MotorConst.turretDown);
                break;
            case PRE_BUCKET_DROP:
                setTurret(MotorConst.turretBucketPreDrop);
                break;
            case SPECIMEN_PRE_PICK:
                setTurret(MotorConst.turretSpecimenPrePick);
                break;
            case SPECIMEN_PRE_DROP:
                setTurret(MotorConst.turretSpecimenPreDrop);
                break;
            case SPECIMEN_DROP:
                setTurret(MotorConst.turretSpecimenDrop);
                break;
            case PRE_HANG:
                setTurret(MotorConst.turretPreHang);
                break;
            case HANG:
                setTurret(MotorConst.turretHang);
                break;
            case SPECIMEN_FRONT_DROP:
                setTurret(MotorConst.turretSpecimenFrontDrop);
                break;
        }
        turretState = state;
    }

    public void setExt(int target){
//        int bottom_clip = (int)(robot.turret.getCurrentPosition()/1250*160);
//        int target_clip = Math.max(Math.min(target - bottom_clip,3000),-bottom_clip);
        robot.extLeft.setTargetPosition(target);
        robot.extRight.setTargetPosition(target);
        robot.extLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extLeft.setPower(MotorConst.extPower);
        robot.extRight.setPower(MotorConst.extPower);
    }

    public void setExtInch(double inch_target){

        int target = (int)(inch_target * 2420/23);
        robot.extLeft.setTargetPosition(target);
        robot.extRight.setTargetPosition(target);
        robot.extLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extLeft.setPower(MotorConst.extPower);
        robot.extRight.setPower(MotorConst.extPower);
    }

    public void setTurret(int target){
        target = Math.max(Math.min(target,1300),0);
        robot.turret.setTargetPosition(target);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(MotorConst.turretPower);
    }

    public void setTurretMotionProfile(int targetPosition, double maxPower) {
        this.targetPosition = Range.clip(targetPosition, MotorConst.turretUp, MotorConst.turretDown);
        this.maxPower = maxPower;

        int currentPosition = robot.turret.getCurrentPosition();
        this.distanceToMove = Math.abs(this.targetPosition - currentPosition);

        this.rampUpDistance = distanceToMove * 0.3;
        this.rampDownDistance = distanceToMove * 0.4;

        robot.turret.setTargetPosition(this.targetPosition);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.isMotionProfileActive = true;
    }

    public void updateTurretMotionProfile() {
        if (!isMotionProfileActive) {
            return;
        }

        int currentPos = robot.turret.getCurrentPosition();
        int distanceRemaining = Math.abs(targetPosition - currentPos);

        double power;

        if (distanceRemaining > distanceToMove - rampUpDistance) {
            double rampUpRatio = (distanceToMove - distanceRemaining) / rampUpDistance;
            power = maxPower * rampUpRatio;
        } else if (distanceRemaining < rampDownDistance) {
            double rampDownRatio = (double) distanceRemaining / rampDownDistance;
            power = maxPower * rampDownRatio;
        } else {
            power = maxPower;
        }

        robot.turret.setPower(Range.clip(power, 0.1, maxPower));

        if (!robot.turret.isBusy() || Math.abs(robot.turret.getCurrentPosition() - targetPosition) > 20) {
            robot.turret.setTargetPosition(this.targetPosition);
            robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.setPower(maxPower);
        }
    }

    public boolean isExtBusy(){
        return robot.extRight.isBusy();
    }

    public boolean isTurretBusy(){
        return robot.turret.isBusy();
    }

    public int TurretPos(){
        return robot.turret.getCurrentPosition();
    }

    public int ExtPos(){
        return robot.extLeft.getCurrentPosition();
    }
}
