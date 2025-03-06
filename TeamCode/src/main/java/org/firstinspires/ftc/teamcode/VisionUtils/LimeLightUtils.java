package org.firstinspires.ftc.teamcode.VisionUtils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class LimeLightUtils {
    public Limelight3A limelight3A;
    public LimeLightUtils(HardwareMap hardwareMap){
        this.limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
    }

    public List<Sample> red_samples = new ArrayList<>();
    public List<Sample> yellow_samples = new ArrayList<>();
    public List<Sample> blue_samples = new ArrayList<>();
    public List<Sample> yellow_and_blue_samples = new ArrayList<>();
    public List<Sample> yellow_and_red_samples = new ArrayList<>();
    public List<Sample> relevantSamples = new ArrayList<>();
    PerspectiveSolver perspectiveSolver = new PerspectiveSolver();
    KinematicSolver kinematicSolver = new KinematicSolver();
    public double ext_target = 0;
    public Arm.WristState wristState = Arm.WristState.WRIST0;
    public double yaw_target = ServoConst.yawNeutral;

    public void filter(LLResult result,Telemetry telemetry){
        try{
            for (LLResultTypes.DetectorResult detector : result.getDetectorResults()) {
                List<Double> pt1 = detector.getTargetCorners().get(0);
                List<Double> pt2 = detector.getTargetCorners().get(1);
                List<Double> pt3 = detector.getTargetCorners().get(2);
                List<Double> pt4 = detector.getTargetCorners().get(3);

                double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
                double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
                double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
                double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

                double width = max_x - min_x;
                double height = max_y - min_y;

                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

                Point field_pos = perspectiveSolver.getX2Y2(new Point(cx, cy),width/height);

                boolean orientation = width/height>VisionConst.wh_threshold;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();
                double x_offset = offset_from_wh_ratio(width/height);
                double y_offset = y_offset_from_field_pos(field_pos.y);
                int preference = 99;
                if (width/height>0.8 && width/height<1.45) preference = 2;
                else preference =1;
//                double y_offset = 0;
                if (field_pos.y > VisionConst.y_range_neg && field_pos.y<VisionConst.y_range_pos && field_pos.x < VisionConst.x_range_max && field_pos.x>VisionConst.x_range_min && class_id == 0) {
                    blue_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    yellow_and_blue_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    relevantSamples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));

                }
                if (field_pos.y > VisionConst.y_range_neg && field_pos.y<VisionConst.y_range_pos && field_pos.x < VisionConst.x_range_max && field_pos.x>VisionConst.x_range_min && class_id == 1) {
                    red_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    yellow_and_red_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    relevantSamples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                }
                if (field_pos.y > VisionConst.y_range_neg && field_pos.y<VisionConst.y_range_pos && field_pos.x < VisionConst.x_range_max && field_pos.x>VisionConst.x_range_min && class_id == 2) {
                    yellow_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    yellow_and_blue_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    yellow_and_red_samples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                    relevantSamples.add(new Sample(field_pos,class_id,confidence,orientation,width/height,preference));
                }

                ////////Custom Filters//////////
//                relevantSamples.removeIf(o->!(o.wh_ratio >=0.5 && o.wh_ratio <=0.86) || (o.wh_ratio>=1.3 && o.wh_ratio<=1.65));

                Comparator<Sample> comparator = Comparator.comparingDouble(sample->sample.preference);
                comparator = comparator.thenComparingDouble(sample->Math.abs(sample.field_pos.x-11));
                relevantSamples.sort(comparator);
                yellow_samples.sort(comparator);
                red_samples.sort(comparator);
                blue_samples.sort(comparator);
                yellow_and_blue_samples.sort(comparator);
                yellow_and_red_samples.sort(comparator);
            }
        }
        catch (Exception e){
            telemetry.addData("Failed to input samples:",e);
        }


    }
    public void filter_color(LLResult result, Telemetry telemetry, String color){
        try{
            for (LLResultTypes.DetectorResult detector : result.getDetectorResults()) {
                List<Double> pt1 = detector.getTargetCorners().get(0);
                List<Double> pt2 = detector.getTargetCorners().get(1);
                List<Double> pt3 = detector.getTargetCorners().get(2);
                List<Double> pt4 = detector.getTargetCorners().get(3);

                double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
                double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
                double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
                double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

                double width = max_x - min_x;
                double height = max_y - min_y;

                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

                Point field_pos = perspectiveSolver.getX2Y2(new Point(cx, cy),width/height);

                boolean orientation = width/height>VisionConst.wh_threshold;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();

                double x_offset = offset_from_wh_ratio(width/height);
                double y_offset = y_offset_from_field_pos(field_pos.y);

                int preference;
                if (width/height>0.8 && width/height<1.4) preference = 2;
                else preference =1;

                if(color.equals("BlueYellow") || color.equals("Blue")) {
                    if (field_pos.y > VisionConst.y_range_neg && field_pos.y < VisionConst.y_range_pos && field_pos.x < VisionConst.x_range_max && field_pos.x > VisionConst.x_range_min && (class_id == 0 || class_id == 2)) {
                        blue_samples.add(new Sample(field_pos, 0, confidence, orientation, width / height,preference));
//                        yellow_and_blue_samples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
                        relevantSamples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
                    }
                }
                if(color.equals("RedYellow")|| color.equals("Red")) {
                    if (field_pos.y > VisionConst.y_range_neg && field_pos.y < VisionConst.y_range_pos && field_pos.x < VisionConst.x_range_max && field_pos.x > VisionConst.x_range_min && (class_id == 1 || class_id == 2)) {
                        red_samples.add(new Sample(field_pos, 1, confidence, orientation, width / height,preference));
                        relevantSamples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
                    }
                }

                if(color.equals("Yellow")) {
                    if (field_pos.y > VisionConst.y_range_neg && field_pos.y < VisionConst.y_range_pos && field_pos.x < VisionConst.x_range_max && field_pos.x > VisionConst.x_range_min && class_id == 2) {
//                        yellow_samples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
//                        yellow_and_blue_samples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
//                        yellow_and_red_samples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
                        relevantSamples.add(new Sample(field_pos, class_id, confidence, orientation, width / height,preference));
                    }
                }

                ////////Custom Filters//////////
                Comparator<Sample> comparator = Comparator.comparingDouble(sample->sample.preference);
//                comparator = comparator.thenComparingDouble(sample->Math.floor(Math.abs(sample.field_pos.y)));
                comparator = comparator.thenComparingDouble(sample->sample.field_pos.x);
                relevantSamples.sort(comparator);
                yellow_samples.sort(comparator);
                red_samples.sort(comparator);
                blue_samples.sort(comparator);
            }
        }
        catch (Exception e){
            telemetry.addData("Failed to input samples:",e);
        }


    }
    public void set_target(Point target){
        double[] solverOut = kinematicSolver.getExtYaw(target);
        ext_target = solverOut[0];
        yaw_target = solverOut[1];
    }

    public static class PerspectiveSolver{

        //Calculates coordinates with respect to bottom of camera frame using cross ratio
        //Input any 3 reference points in camera and their respective position in global frame

        /*Cross ratio = AC x BD / AD x BC, the variable input being B in this case
        We solve for field pos by equating the cross ratio as k = A'C' x B'D'/A'D' x B'C',
        where B' is the unknown.
        Final equation - k' = (AC x BD/AD x BC)*(A'D'/A'C') = (B'D'/B'C')*/

        double camera_angle;
        int CAMERA_HEIGHT;
        int CAMERA_WIDTH;
        double x_offset,y_offset,camera_offset;
        double cx1,cx2,cx3,fx1,fx2,fx3,w1,w2;
        double cy1,cy2,cy3,fy1,fy2,fy3;
        CameraOrientation orientation;


        public PerspectiveSolver() {
            this.camera_angle = Math.toRadians(VisionConst.camera_angle);
            this.x_offset = VisionConst.x_offset;
            this.y_offset = VisionConst.y_offset;
            this.camera_offset = VisionConst.camera_offset;
            this.cx1 = VisionConst.cx1;
            this.cx2 = VisionConst.cx2;
            this.cx3 = VisionConst.cx3;
            this.fx1 = VisionConst.fx1;
            this.fx2 = VisionConst.fx2;
            this.fx3 = VisionConst.fx3;
            this.cy1 = VisionConst.cy1;
            this.cy2 = VisionConst.cy2;
            this.cy3 = VisionConst.cy3;
            this.fy1 = VisionConst.fy1;
            this.fy2 = VisionConst.fy2;
            this.fy3 = VisionConst.fy3;
            this.w1 = VisionConst.w1;
            this.w2 = VisionConst.w2;
            this.CAMERA_HEIGHT = VisionConst.CAMERA_HEIGHT;
            this.CAMERA_WIDTH = VisionConst.CAMERA_WIDTH;
            this.orientation = VisionConst.orientation;
        }

        public Point getX2Y2(Point ObjectPose, double wh){
            //Solver for 2nd point in the points ordered A, B, C and D.

            double cam_x = CAMERA_HEIGHT - ObjectPose.y;
            double cam_y = CAMERA_WIDTH/2.0 - ObjectPose.x;

            double field_x,field_y;

            if (cam_x==cx2){
                field_x = fx2 + camera_offset;
                //width is w1 at x1 and w2 at x2
                double width = w1 + (w2-w1)/(fx2-fx1)*(fx2-fx1);
                field_y = cam_y/CAMERA_WIDTH*width;
            }
            else{

                double kx = ((cx2-cx1)*(cx3 - cam_x))/((cx3 - cx1)*(cx2 - cam_x))*(fx3-fx1)/(fx2-fx1);
                field_x = (kx*fx2-fx3)/(kx-1) + camera_offset;
                double width = w1 + (w2-w1)/(fx2-fx1)*(field_x-fx1);

                double ky = ((cy2-cy1)*(cy3 - cam_y))/((cy3 - cy1)*(cy2 - cam_y))*(fy3-fy1)/(fy2-fy1);
                field_y = (ky*fy2-fy3)/(ky-1)*width/w1;
//                field_y = cam_y/CAMERA_WIDTH*width;
            }
            
            field_x += offset_from_wh_ratio(wh);
            field_y += y_offset_from_field_pos(field_y);

            double field_x_tr = field_x*Math.cos(camera_angle) - field_y*Math.sin(camera_angle) + x_offset;
            double field_y_tr = field_x*Math.sin(camera_angle) + field_y*Math.cos(camera_angle) + y_offset;
            return new Point(field_x_tr,field_y_tr);
        }
    }
    public static class KinematicSolver {
        double arm_l1;
        double arm_l2;
        double x_offset;
        double y_offset;
        double theta_offset;

        public KinematicSolver(){
            this.arm_l1 = VisionConst.arm_l1;
            this.arm_l2 = VisionConst.arm_l2;
            this.x_offset = VisionConst.x_offset_k;
            this.y_offset = VisionConst.y_offset_k;
            this.theta_offset = Math.atan(arm_l2/arm_l1);  //verify this if viper behave diff
        }

        //takes target point as input and gives extension in inches and yaw in degrees as element 0 and 1 of the array
        public double[] getExtYaw(Point target){
            double theta = 0;
            double ext = 0;
            double arm_l = Math.sqrt(arm_l1*arm_l1 + arm_l2*arm_l2);
            if(Math.abs(target.y+y_offset) <= arm_l) {
                theta = Math.toDegrees(Math.asin((target.y + y_offset) / (arm_l)) + theta_offset);
                ext = target.x + x_offset - arm_l * Math.cos(Math.toRadians(theta));
            }


            ext = Range.clip(ext,0,VisionConst.bot_ext_limit);
            theta = Range.clip(theta, VisionConst.bot_neg_yaw_limit, VisionConst.bot_pos_yaw_limit);
            return new double[]{ext,theta};
        }
    }
    public static class Sample{
        public double confidence;
        public int class_id;
        public Point field_pos;
        public boolean orientation;
        public double wh_ratio;
        public int preference;

        public Sample(Point field_pos,int class_id,double confidence,boolean orientation,double wh_ratio,int preference){
            this.field_pos = field_pos;
            this.class_id = class_id;
            this.confidence = confidence;
            this.orientation = orientation;
            this.wh_ratio = wh_ratio;
            this.preference = preference;
        }
    }
    public enum CameraOrientation {
        UPRIGHT(0),
        UPSIDE_DOWN(1),
        RIGHT_90(2),
        LEFT_90(3);
        public final int x;

        CameraOrientation(int x){this.x = x;}

        public int getOrientation(){return x;}

    }





    //TODO LIMELIGHT SETUP

    public void print_samples(Telemetry telemetry){
        if(!red_samples.isEmpty()) {
            telemetry.addData("Red Samples",red_samples.size());
            telemetry.addData("Sample",red_samples.get(0).field_pos);
            telemetry.addData("Orientation",red_samples.get(0).orientation);
            telemetry.addData("WH",red_samples.get(0).wh_ratio);
        }
        else telemetry.addData("Red Samples",0);

        if(!blue_samples.isEmpty())  {
            telemetry.addData("Blue Samples",blue_samples.size());
            telemetry.addData("Sample",blue_samples.get(0).field_pos);
            telemetry.addData("Orientation",blue_samples.get(0).orientation);
            telemetry.addData("WH",blue_samples.get(0).wh_ratio);
        }
        else telemetry.addData("Blue Samples",0);

        if(!yellow_samples.isEmpty())  {
            telemetry.addData("Yellow Samples",yellow_samples.size());
            telemetry.addData("Sample",yellow_samples.get(0).field_pos);
            telemetry.addData("Orientation",yellow_samples.get(0).orientation);
            telemetry.addData("WH",yellow_samples.get(0).wh_ratio);
        }
        else telemetry.addData("Yellow Samples",0);
    }
    public void clear_samples(){
        yellow_samples.clear();
        red_samples.clear();
        blue_samples.clear();
        yellow_and_red_samples.clear();
        yellow_and_blue_samples.clear();
        relevantSamples.clear();
    }

    //TODO GIVES

    public static double offset_from_wh_ratio(double wh_ratio){
        double a = 1.78716987;
        double b = -7.18326796;
        double c = 7.83406123;
        double d = -0.61362127;

        double offset = a*wh_ratio*wh_ratio*wh_ratio + b*wh_ratio*wh_ratio + c*wh_ratio + d;
        return Range.clip(offset,0.5,2);
    }
    public static double y_offset_from_field_pos(double pos){
        double a = -1.15347454e-03;
        double b = 0;//Very small
        double c = -1.15312523e-01;
        double d = 0;//Very small

        double offset = a*pos*pos*pos + b*pos*pos + c*pos + d;
        return Range.clip(offset,-1.5,1.5);
    }



}
