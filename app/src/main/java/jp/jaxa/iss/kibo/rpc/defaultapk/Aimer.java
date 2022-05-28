package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import java.lang.*;

import java.util.Arrays;

public class Aimer {

    private final double arTag_sizePx;
    private final double meter_perPx;
    private double laser2target_x = 0;
    private double laser2target_y = 0;

    private final double laser_depth  =  0.1302;
    private final double laser_width = 0.0572;
    private final double laser_high = 0.1111;
    private final double cam_width  = -0.0422;
    private final double cam_high  = 0.0826;


    public Aimer(double arTag_sizePx , double  meter_perPx ) {
        this.arTag_sizePx = arTag_sizePx;
        this.meter_perPx = meter_perPx;

    }

    public double[] pixelDistanceToAngle(double[] targetPosition ) {
        final String TAG = "pixelDistanceToAngle";
        Log.i(TAG, "================== pixelDistanceToAngle ==================");
        final double[] ref  = {635.434258, 500.335102}; // original
        final double focusCamera = 4.161542;
        final double cam_depth = 0.1177;

        final double xOffset = -0.011;
        final double yOffset = -0.053;

        double cam2walldis = focusCamera * 5 / arTag_sizePx;
        double l2w =  cam2walldis - ( laser_depth - cam_depth);

        Log.i(TAG, "cam2walldis=" + cam2walldis);
        Log.i(TAG, "l2w = " + l2w);

        double cam_target_px_x = (targetPosition[0]) - ref[0];
        double cam_target_px_y = ref[1] - (targetPosition[1]);
        Log.i(TAG, "cam_target_px_x=" + cam_target_px_x);
        Log.i(TAG, "cam_target_px_y=" + cam_target_px_y);

        double cam_target_dis_x = ( cam_target_px_x * meter_perPx ) ;
        double cam_target_dis_y = ( cam_target_px_y * meter_perPx ) ;
        Log.i(TAG, "cam_target_dis_x=" + cam_target_dis_x);
        Log.i(TAG, "cam_target_dis_y=" + cam_target_dis_y);

        laser2target_x = cam_target_dis_x -  ( cam_width +  laser_width );
        laser2target_y = cam_target_dis_y - (laser_high - cam_high );
        Log.i(TAG, "laser2target_x=" + laser2target_x);
        Log.i(TAG, "laser2target_y=" + laser2target_y);

        double centerKibo2target_x = cam_target_dis_x + cam_width + xOffset;
        double centerKibo2target_y = cam_target_dis_y + cam_high - yOffset;
        Log.i(TAG, "centerKibo2target_x (m)= " + centerKibo2target_x);
        Log.i(TAG, "centerKibo2target_y (m)= " + centerKibo2target_y);

        //########  horizonAngle axis #######
        double horizonAngle = horizonAngle_axis(l2w, centerKibo2target_x);
        Log.i(TAG, "horizonAngle(x)= " + horizonAngle);
        //###################################

        //########  verticalAngle axis #######
        double verticalAngle = verticalAngle_axis(l2w, centerKibo2target_y);
        Log.i(TAG, "verticalAngle(y)= " + verticalAngle);
        //###################################

        double[] out = {horizonAngle, verticalAngle};
        Log.i(TAG, "Angle(x,y)= " + Arrays.toString(out));
        Log.i(TAG, "================== Done ==================");
        return out;
    }


    // horizon angle
    private double horizonAngle_axis(double l2t ,double xc) {
        String TAG = "horizonAngle_axis";


        double laser_oblique_x = 0.14221068876846074;

        // Angle calculation from tools/laser.ipynb
        final double pivotAngle = 2.727652176143348;

        // Length from Astrobee pivot to target
        double l = Math.sqrt(Math.pow(xc, 2) + Math.pow(l2t, 2));
        double lp = Math.sqrt(Math.pow(xc - laser_width, 2) + Math.pow(l2t - laser_depth, 2));

        double angle1  = Math.acos((Math.pow(laser_oblique_x, 2) + Math.pow(l, 2) - Math.pow(lp, 2))/(2* laser_oblique_x *l));
        double angle2  = Math.toRadians(180) - pivotAngle - Math.asin((laser_oblique_x *Math.sin(pivotAngle))/l);

        Log.i(TAG, "l = " + l);
        Log.i(TAG, "lp = " + lp);
        Log.i(TAG, "angle1 = " + angle1);
        Log.i(TAG, "angle2 = " + angle2);

        double horizonAngle = (angle1 - angle2);

        if(laser2target_x < 0){
            Log.i(TAG, "target on left laser ");
            Log.i(TAG, "horizonAngle are negative");
            horizonAngle =   horizonAngle * -1;
        }
        Log.i(TAG, "turn_angle = " + horizonAngle);

        return horizonAngle;
    }

    // vertical angle
    private double verticalAngle_axis(double l2t, double yc) {
        String TAG = "verticalAngle_axis";

        double laser_oblique_y = 0.1711585522257068;

        // Angle calculation from tools/laser.ipynb
        final double pivotAngle = 2.435184375290124;

        // Length from Astrobee pivot to target
        double l = Math.sqrt(Math.pow(l2t, 2) + Math.pow(yc, 2));
        double lp = Math.sqrt(Math.pow(l2t-laser_depth, 2) + Math.pow(yc- laser_high, 2));

        double angle1  = Math.acos((Math.pow(laser_oblique_y, 2) + Math.pow(l, 2) - Math.pow(lp, 2))/(2* laser_oblique_y *l));
        double angle2  = Math.toRadians(180) - pivotAngle - Math.asin((laser_oblique_y *Math.sin(pivotAngle))/l);

        Log.i(TAG, "l = " + l);
        Log.i(TAG, "lp = " + lp);
        Log.i(TAG, "angle1 = " + angle1);
        Log.i(TAG, "angle2 = " + angle2);

        double verticalAngle = (angle1 - angle2);

        if(laser2target_y < 0){
            Log.i(TAG, "target lower than laser ");
            Log.i(TAG, "verticalAngle are negative");
            verticalAngle =   verticalAngle * -1;
        }
        Log.i(TAG, "turn_angle = " + verticalAngle);

        return verticalAngle;
    }

}
