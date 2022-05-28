package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;

import org.opencv.core.CvType;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.undistort;

public class NavCam {

    private final String FailTAG = "Fail_NavCam";
    private static boolean FAIL_to_Find_TARGET = false;

    private final int cropPic_x = 350;
    private final int cropPic_y = 350;
    private static double arTag_sizePx;
    private static double meter_perPx;

    private final int NAV_LOOP_MAX = 4;


    public NavCam(){
    }
    class arTag_data {
        double size_x, size_y, imagePoint_x, imagePoint_y;

        arTag_data(double size_x, double size_y, double imagePoint_x, double imagePoint_y) {
            this.size_x = size_x;
            this.size_y = size_y;
            this.imagePoint_x = imagePoint_x;
            this.imagePoint_y = imagePoint_y;
        }
    }

    private arTag_data getArTag_data (Mat corners) { // find center && distance
        final String TAG = "getArTag_data";
        double xCenter;
        double yCenter;
        double xLeft=0;
        double xRight=0;
        double yUp=0;
        double yDown=0;
        double AR_xDist;
        double AR_yDist;

        for(int i =0 ; i<4 ; i++) {
            Log.i(TAG, "corners: " +i  + " corners_x:" +corners.get(0, i)[0] + " corners_y:" +corners.get(0, i)[1] );
        }

        xCenter = (corners.get(0, 0)[0] + corners.get(0, 1)[0] + corners.get(0, 2)[0] + corners.get(0, 3)[0]) / 4.0f;
        yCenter = (corners.get(0, 0)[1] + corners.get(0, 1)[1] + corners.get(0, 2)[1] + corners.get(0, 3)[1]) / 4.0f;

        // ########### find arTag size in undistort ################
        for(int i =0 ; i<4 ; i++){      // for X axis
            if(corners.get(0, i)[0] < xCenter){
                xLeft +=corners.get(0, i)[0];
            }else
                xRight += corners.get(0, i)[0];

        }
        for (int i = 0 ; i<4 ; i++){ // for Y axis
            if(corners.get(0, i)[1] < yCenter){
                yUp+=corners.get(0, i)[1];
            }else {
                yDown+=corners.get(0, i)[1];
            }
        }
        AR_xDist = (xRight-xLeft)/2;
        AR_yDist = (yDown-yUp)/2;
        // ###########################
        Log.i(TAG, " AR_xDist:"  + AR_xDist + " AR_yDist:" + AR_yDist );
        return new arTag_data(AR_xDist, AR_yDist, xCenter, yCenter);
    }

    public Mat findARtag(Mat undistortPic) {
        final String TAG = "ar_read";
        final long start = System.currentTimeMillis();
        final Dictionary bluePrint = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Log.i(TAG, "Reading AR");

        int counter = 0;
        Mat cropPic = new Mat(undistortPic, new Rect(cropPic_x, cropPic_y, 600, 360));

        while (ids.rows() != 4 && counter < NAV_LOOP_MAX) { // try 4 until find all 4 ids
            //############ detect Markers ############
            Aruco.detectMarkers(cropPic, bluePrint, corners, ids); // find all ids
            //########################
            counter++;
        }
        Log.i(TAG, "ids= " + ids.dump());
        long end = System.currentTimeMillis();
        Log.i(TAG, "ar_read_time=" + (end-start));

        arTag_data[] arTag = new  arTag_data[4];

        if(ids.rows() == 4) {
            Log.i(TAG, "All 4 ids are found.");
            double ar_x_avg_px=0;
            double ar_y_avg_px=0;

            for (int i = 0; i < 4; i++) {
                arTag[i] =  getArTag_data(corners.get(i));

                Log.i(TAG, "imagePoint_x at "+ i +":"+ + (int)arTag[i].imagePoint_x + ",imagePoint_y at " + i +":"+ (int) arTag[i].imagePoint_y  );
                ar_x_avg_px += arTag[i].size_x;
                ar_y_avg_px += arTag[i].size_y;
            }

            Mat detectedMarkerImg = cropPic.clone();
            Aruco.drawDetectedMarkers(detectedMarkerImg,corners, ids, new Scalar(0,255,255) );

            arTag_sizePx = (ar_x_avg_px/4 + ar_y_avg_px/4)  / 2  ;

            meter_perPx = 0.05 / arTag_sizePx;

            Log.i(TAG, "arTag_sizePx : " + arTag_sizePx );
            Log.i(TAG, "meter_perPx : " + meter_perPx );
            Log.i(TAG, "Center x : " + (arTag[0].imagePoint_x + arTag[1].imagePoint_x + arTag[2].imagePoint_x + arTag[3].imagePoint_x) / 4.0f );
            Log.i(TAG, "Center y : " + (arTag[0].imagePoint_y + arTag[1].imagePoint_y + arTag[2].imagePoint_y + arTag[3].imagePoint_y ) / 4.0f );

            Mat Center_Target = findCenterTarget_Cycle(cropPic);
            Log.i(TAG, "Targrt At =" + Center_Target.dump() );

            return  Center_Target;
        } else {
            FAIL_to_Find_TARGET = true;
            Log.i(FailTAG, "AR detectMarkers");
            Log.i(TAG, "--Fail: Only found " + ids.rows() + " markers");
            return undistortPic;
        }
    }


    private Mat findCenterTarget_Cycle(Mat pic) {
        final String TAG = "findTargetCenter";
        Mat circles = new Mat();

        Mat out = new Mat(1, 1, CvType.CV_32FC2);

        try {
            Imgproc.HoughCircles(pic, circles, Imgproc.HOUGH_GRADIENT, 1.0, 300.0, 50.0, 30.0, 40, 50);
//            Imgproc.HoughCircles(pic, circles, Imgproc.HOUGH_GRADIENT, 1.0,
//                    (double) pic.rows() / 16, // change this value to detect circles with different distances to each other
//                    100.0, 30.0, 1, 30); // change the last two parameters
//            // (min_radius & max_radius) to detect larger circles
            Log.i(TAG, "Succeed HoughCircles");
            // circle center
            Mat ArucoDetectedCenter = pic.clone();

            double[] c = circles.get(0, 0);
            org.opencv.core.Point center = new org.opencv.core.Point(Math.round(c[0]), Math.round(c[1]));

            Imgproc.circle(ArucoDetectedCenter, center, 1, new Scalar(255, 0, 255), 3, 8, 0);
//            api.saveMatImage(ArucoDetectedCenter, (System.currentTimeMillis() - debug_Timestart) + " ArucoDetectedCenter.png");

            double[] point = {center.x + cropPic_x, center.y + cropPic_y};
            out.put(0, 0, point);
            Log.i(TAG, "Succeed findTargetCenter =" + out.dump());
            return out;

        }catch (Exception e){
            //######################
            FAIL_to_Find_TARGET = true;
            //######################
            Log.i(FailTAG, "findTargetCenter_Cycle =" + e);
            double[] point = { -1 , -1};
            out.put(0, 0, point);
            return out;
        }

    }


    public Mat undistortPicture(Mat pic) {
        final String TAG = "undistortPic";
//        api.saveMatImage(pic,  (System.currentTimeMillis()-debug_Timestart ) + " Input Pic to undistort.png" );
        final double[] CAM_MATSIM = {
                523.105750, 0.0, 635.434258,
                0.0, 534.765913, 500.335102,
                0.0, 0.0, 1.0
        };

        final double[] DIST_COEFFSIM = {
                -0.164787, 0.020375, -0.001572, -0.000369, 0.0
        };

        Log.i(TAG, "Start");

        try {
            Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
            Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

            cameraMat.put(0, 0, CAM_MATSIM);
            distCoeffs.put(0, 0, DIST_COEFFSIM);
            Mat out = new Mat(pic.rows(), pic.cols(), pic.type());

            undistort(pic, out, cameraMat,distCoeffs);
//            api.saveMatImage(out,  (System.currentTimeMillis()-debug_Timestart ) + " undistort pic.png");
            Log.i(TAG, "Succeed undistort  "  );
            return out;
        }catch (Exception e){
            Log.i(FailTAG, "undistortPic =" + e);
            Log.i(TAG, "Fail undistort " + e );
            return pic;
        }
    }

    public static double getArTag_sizePx(){
        return arTag_sizePx;
    }
    public static double getMeter_perPx(){
        return meter_perPx;
    }
    public static boolean FailtoFindTarget(){
        return FAIL_to_Find_TARGET;
    }
}
