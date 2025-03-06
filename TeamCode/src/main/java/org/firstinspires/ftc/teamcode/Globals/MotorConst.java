package org.firstinspires.ftc.teamcode.Globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorConst {

    public static int extInit = 0;
    public static int extMin = 0;
    public static int extMax = 2900;
    public static int extHorizontalMax = 1200;
    public static int extHighBucketDrop = 2850;
    public static int extSpecimenPrePick = 0;
    public static int extSpecimenPreDrop = 650;
    public static int extSpecimenDrop = 1100;
    public static int extPreHang = 1400;
    public static int extHang = 150;
    public static int extFirstSpecimen = 500;
    public static int extSpecimenFrontDrop = 1050;

    public static int turretInit = 0;
    public static int turretUp = 0;
    public static int turretDown = 1250;
    public static int turretBucketPreDrop = 0;
    public static int turretSpecimenPrePick = 460;
    public static int turretSpecimenPreDrop = 0;
    public static int turretSpecimenDrop = 0;
    public static int turretPreHang = 550;
    public static int turretHang = 620;
    public static int turretSpecimenFrontDrop = 710;

    public static int hangerInit = 0;
    public static int hangerUpPhase1 = 1100;
    public static int hangerDownPhase1 = 400;

    public static double extPower = 1;
    public static double turretPower = 1;

    public static double turretTimeConst = 0.55;
    public static double extTimeConst = 1.3;
}
