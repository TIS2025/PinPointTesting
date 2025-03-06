package org.firstinspires.ftc.teamcode.VisionUtils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConst {
    ///////////////Perspective Solver//////////////
    public static int CAMERA_HEIGHT = 480;
    public static int CAMERA_WIDTH = 640;

    public static double camera_angle = 25;
    public static double x_offset = -0.87;
    public static double y_offset = -4.15;
    public static double camera_offset = 3.190 + 1.05;
    public static double cx1 = 0;
    public static double cx2 = CAMERA_HEIGHT-350;
    public static double cx3 = CAMERA_HEIGHT - 200;
    public static double fx1 = 0;
    public static double fx2 = 2.651;
    public static double fx3 = fx2 + 4.57;

    public static double w1 = 5.484 + 5.484;
    public static double w2 = 6.332 + 6.405;

    //Non-Tuned Value for matrix
    public static double cy1 = 0;
    public static double cy2 = 100;
    public static double cy3 = 200;
    public static double fy1 = 0;
    public static double fy2 = 1.68;
    public static double fy3 = fy2 + 1.63;
    public static LimeLightUtils.CameraOrientation orientation = LimeLightUtils.CameraOrientation.UPRIGHT;


    ///////////////Kinematic Solver///////////////
    public static double arm_l1=4.5;
    public static double arm_l2=0;
    public static double x_offset_k = -0.8;
    public static double y_offset_k = 0.5;

    ///////////////Sample Filtering//////////////
    public static double x_range_max = 18;
    public static double x_range_min = 6;
    public static double y_range_neg = -4;
    public static double y_range_pos = 4;
    public static double wh_threshold = 1.1;

    //////////////Range Limits///////////////////
    public static double bot_ext_limit = 15;
    public static double bot_neg_yaw_limit = -80;
    public static double bot_pos_yaw_limit = 80;
}
