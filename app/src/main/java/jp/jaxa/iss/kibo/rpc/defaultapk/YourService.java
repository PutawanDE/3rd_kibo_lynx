package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.atan;
import static java.lang.Math.toDegrees;
import static org.opencv.imgproc.Imgproc.CV_HOUGH_GRADIENT;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.undistort;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 3;
    boolean check_point_A = false;
    boolean check_target_1 = false;
    boolean check_point_B = false;
    boolean check_target_2 = false;

    boolean FAIL_to_Find_TARGET = false;
    boolean It_realworld = false;
    final String FailTAG = "Fail_method";

    boolean debug = true;
    boolean debug_pointB_unaim_snap = false;
    boolean debug_pointB_aim_snap = false;
    long debug_Timestart = 0;

    double arTag_sizePx= -1;
    double meter_perPx= -1;

    double laser_width  =  0.0572;
    double laser_depth  =  0.1302;
    double laser_oblique_x = 0.14221;
    double laser_oblique_y = 0.171159;
    double laser_high = 0.1111;

    double cam_depth = 0.1177;
    double cam_width  = 0.0422;
    double cam_high  = 0.0826;


    final Point point_A_to_shoot_target1 = new Point(10.71, -7.811971 , 4.48);
    final Point point_B_to_shoot_target2 = new Point(11.2746, -9.92284, 5.29881); // original
//    final Point point_B_to_shoot_target2 = new Point(11.2746 + 0.089, -9.92284, 5.29881 + 0.165);  // move right 8.9cm down 16.cm    Point(11.3636, -9.92284, 5.46381);
    final Point point_Goal_target = new Point(11.2746, -7.89178, 4.96538);

    @Override
    protected void runPlan1(){
        debug_Timestart = System.currentTimeMillis();
        api.startMission();
        
        // move to point A
        check_point_A = mission_point_A();

        // shoot point 1
        check_target_1 = mission_target_1();

        // move to point A'
        check_point_B = mission_point_B();

//        mission_test();

        // shoot point 2
        check_target_2 = mission_target_2();

        mission_report();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    private void  mission_test(){
        try {
            final Point point_20cm = new Point(11.0746, -9.92284, 5.29881);
            final Point point_90cm = new Point(11.2746, -9.92284, 5.29881);

            final Quaternion q_B = new Quaternion(0f, 0f, -0.707f, 0.707f);
            Thread.sleep(10000);
            for (int i = 0 ; i < 2 ; i++) {
                move_to(point_90cm, q_B);
                api.laserControl(true);
                api.takeTarget1Snapshot();
                Thread.sleep(5000);
                Bitmap laserPic = api.getBitmapNavCam();
                api.saveBitmapImage(laserPic, "laser targer 1 at B");
                api.laserControl(false);

                Thread.sleep(10000);

                move_to(point_20cm, q_B);
                api.laserControl(true);
                api.takeTarget1Snapshot();
                Thread.sleep(5000);
                Bitmap Pic_50 = api.getBitmapNavCam();
                api.saveBitmapImage(Pic_50, "laser targer 2 at B");
                api.laserControl(false);

                Thread.sleep(5000);

            }
        }catch (Exception e ){
            Log.i(FailTAG, "mission_test =" + e);
        }

    }

    private boolean mission_point_A(){
        final Point point_A = new Point(10.71000, -7.70000, 4.48000);
        final Quaternion q_A = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);
        boolean point_A_Succeeded;

        try {
            point_A_Succeeded = move_to(point_A, q_A);
            api.reportPoint1Arrival();

            Bitmap image1_A = api.getBitmapNavCam();
            api.saveBitmapImage(image1_A, "Point A Arrival");
            return point_A_Succeeded;
        }catch (Exception e){
            Log.i(FailTAG, "mission_point_A =" + e);
            return  false;
        }
    }

    private boolean mission_target_1(){
         try {
             boolean aim_Succeed = false;

             // AI aim and shoot
             int aim_try = 0;
             do {
                 aim_Succeed = aim_shoot("A");
                 aim_try++;
             }while (!aim_Succeed && aim_try < LOOP_MAX );

            if(aim_Succeed) {
                //#######################
                if(FAIL_to_Find_TARGET && It_realworld){
                    Log.i(FailTAG, "FAIL_to_READ_AR target_1");
                    return  false;
                }
                //#######################
                api.laserControl(true);
                api.takeTarget1Snapshot();

                Bitmap laserPic = api.getBitmapNavCam();
                api.saveBitmapImage(laserPic, "laser targer 1 at A");
                api.laserControl(false);

                return  true;
            }else {
                Log.i(FailTAG, "FAIL_to_aim target_1");
                return  false;
            }

         }catch (Exception e){
             Log.i(FailTAG, "mission_target_1 =" + e);
             return  false;
         }

    }

    private boolean mission_point_B(){
        final Quaternion q_B = new Quaternion(0f, 0f, -0.707f, 0.707f);

        final Point point_B_0 = new Point(10.876214285714285, -8.5, 4.97063);
        final Point point_B_1 = new Point(11.0067, -9.44819, 5.186407142857143);

        boolean point_B_prime_Succeeded;
        try {
            move_to(point_B_0,q_B);
            move_to(point_B_1,q_B);
            point_B_prime_Succeeded = move_to(point_B_to_shoot_target2,  q_B);

            //       save the image
            Bitmap image1_B = api.getBitmapNavCam();
            api.saveBitmapImage(image1_B, "Point B Arrival");

            return  point_B_prime_Succeeded;

        }catch (Exception e){
            Log.i(FailTAG, "mission_point_B =" + e);
            return  false;
        }
    }

    private boolean mission_target_2(){
        final String TAG = "mission_target_2";
        try {
            boolean aim_Succeed = false;
            boolean on_Laser = false;

            for (int i = 0 ; i < 10 ; i++){

                // AI aim and shoot
                //if(i == 0 || i == 5){
                if(i == 0){
                    int aim_try = 0;
                    do {
                        aim_Succeed = aim_shoot("B");
                        aim_try++;
                    }while (!aim_Succeed && aim_try < LOOP_MAX );
                }else {
                    aim_Succeed = true;
                }
                Log.i(TAG, "aim status = "+ i  +" :" + aim_Succeed);
                if(debug){
                    aim_Succeed = true;
                }

                if(aim_Succeed || debug) {
                    //#######################
                    if(FAIL_to_Find_TARGET && It_realworld){
                        Log.i(FailTAG, "FAIL_to_READ_AR target_2");
                        return  false;
                    }
                    //#######################
                    if(!on_Laser){
                        api.laserControl(true);
                        on_Laser = true;
                    }
                    api.takeTarget2Snapshot();

//                    if(i == 0 || i == 5){
                    if(i == 0){

                            Bitmap laserPic = api.getBitmapNavCam();
                        api.saveBitmapImage(laserPic, "laser B try "+i);
                    }
                }else {
                    Log.i(FailTAG, "FAIL_to_aim target_2");
                }

            }
            api.laserControl(false);
            return  true;
        }catch (Exception e){
            Log.i(FailTAG, "mission_target_2 =" + e);
            return  false;
        }
    }

    private void mission_report(){
        final Quaternion q_Goal = new Quaternion(0f, 0f, -0.707f, 0.707f);

        final Point point_Goal_0 = new Point(11.0067, -9.44819, 5.1722);
        final Point point_Goal_1 = new Point(11.073773469387755, -8.5, 4.97063);

        boolean cleared;

        move_to(point_Goal_0,q_Goal);
        move_to(point_Goal_1,q_Goal);
        move_to(point_Goal_target,q_Goal);

        if(debug){
            Boolean[] Mission_State = new  Boolean[]{check_point_A,check_target_1,check_point_B,check_target_2};
            for (int i = 0; i < Mission_State.length; i++) {
                Log.i(Mission_State[i].toString(), "Mission State");
            }
        }

        do {
            cleared = api.reportMissionCompletion();
        } while (!cleared);

    }

    private boolean move_to(Point target_point , Quaternion q) {
        final String TAG = "move_to";
        final Point p = target_point;

        int counter = 0;
        Result result;

        Log.i(TAG, "Start move to p:"  +p +", q:" + q);

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        Log.i(TAG, "Done move to p:"  +p +", q:" + q);
        return  result.hasSucceeded();
    }

    private boolean turn_to(Quaternion q) {
        final String TAG = "move_to";
        final Point p_dormant = new Point(0, 0, 0);

        int counter = 0;
        Result result;

        Log.i(TAG, "Start turn to p:"  + p_dormant +", q:" + q);

        do {
            result = api.relativeMoveTo(p_dormant, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        Log.i(TAG, "Done turn to p:"  +p_dormant +", q:" + q);
        return  result.hasSucceeded();
    }

    private boolean  aim_shoot(String targetType){
        final String TAG = "aim_shoot";
        boolean enable_aim = true;
        try {
            Mat buffer =  api.getMatNavCam();
            api.saveMatImage(buffer,  (System.currentTimeMillis()-debug_Timestart )+" Point B buffer.png" );

            Thread.sleep(1000);
            Mat pic_cam =  api.getMatNavCam();
            Thread.sleep(1000);
            Mat undistort_Cam = undistortPic(pic_cam);

            if (targetType.equals("A")){
                Quaternion q_target1 = new Quaternion(0f, 0.707f, 0f, 0.707f);
//                Point point_to_shoot_target1 = new Point(10.71, -7.811971 , 4.48);
                move_to(point_A_to_shoot_target1, q_target1);
            }
            else if(targetType.equals("B")){

                if(debug && !debug_pointB_unaim_snap){
                    api.saveMatImage(undistort_Cam,  (System.currentTimeMillis()-debug_Timestart )+" Point B unaim.png" );
                    debug_pointB_unaim_snap = true;
                }

//                final Point point_B_target = new Point(11.1352, -9.92284, 5.30881);   // X : -0.0994 - 0.04  , Z + 0.01
                //#######################################
                // FIX Quaternion to shoot target2

                Mat  undistort_AR_Center = cam_ar_read(undistort_Cam);
                double[] target = undistort_AR_Center.get(0, 0);
                Log.i(TAG, "Targrt At =" + "x +350: " + (target[0]+350) +"y +350: " + (target[1]+350)   );

                double[] angleToTurn = pixelDistanceToAngle(undistort_AR_Center.get(0, 0));

//                double[] moveKibo = find_DistanceToMove(angleToTurn);
//                Point point_B_target = new Point(11.2746  , -9.92284, 5.29881 );  // original
//                Point point_B_target = new Point(11.2746  , -9.92284, 5.29881 + moveKibo[1]);  // fix up-down


                Quaternion qToTurn_Target2 = new Quaternion(0, 0, -0.707f, 0.707f);
                if(enable_aim) {
                    Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
                    qToTurn_Target2 = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
                }
                    turn_to(qToTurn_Target2);
                    Log.i(TAG, "aim_shoot Succeed");
                    //#######################################

                if(debug && !debug_pointB_aim_snap){
                    Thread.sleep(1000);
                    Mat pic_aim =  api.getMatNavCam();
                    Thread.sleep(1000);
                    Mat undistort_B_Aim = undistortPic(pic_aim);
                    api.saveMatImage(pic_aim,  (System.currentTimeMillis()-debug_Timestart ) + " Point B aim raw.png");
                    api.saveMatImage(undistort_B_Aim,  (System.currentTimeMillis()-debug_Timestart ) + " Point B aim undis.png");
                    debug_pointB_aim_snap = true;
                }
            }else {
                Log.i(FailTAG, "aim_shoot mode Fail");
            }
            return  true;

        }catch (Exception e){
            Log.i(FailTAG, "aim_shoot =" + e);
            Log.i(TAG, "Fail aim_shoot =" + e);
            return  false;
        }
    }

    static class arTag_data {
        double size_x, size_y, imagePoint_x, imagePoint_y;

        arTag_data(double size_x, double size_y, double imagePoint_x, double imagePoint_y) {
            this.size_x = size_x;
            this.size_y = size_y;
            this.imagePoint_x = imagePoint_x;
            this.imagePoint_y = imagePoint_y;
        }
    }

    private Mat cam_ar_read(Mat undistortPic) {
        final String TAG = "ar_read";
        final long start = System.currentTimeMillis();
        final Dictionary bluePrint = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Log.i(TAG, "Reading AR");

        int counter = 0;
        Mat cropPic = new Mat(undistortPic, new Rect(350, 350, 600, 360));

        while (ids.rows() != 4 && counter < LOOP_MAX) { // try 3 until find all 4 ids
            //############ detect Markers ############
            api.saveMatImage(cropPic,  (System.currentTimeMillis()-debug_Timestart )+ " crop for detect.png");
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
            api.saveMatImage(detectedMarkerImg,  (System.currentTimeMillis()-debug_Timestart )+" ArucoDetectedMarker.png");

            arTag_sizePx = (ar_x_avg_px/4 + ar_y_avg_px/4)  / 2  ;
            meter_perPx = 0.05 / arTag_sizePx;

            Log.i(TAG, "arTag_sizePx : " + arTag_sizePx );
            Log.i(TAG, "meter_perPx : " + meter_perPx );
            Log.i(TAG, "Center x : " + (arTag[0].imagePoint_x + arTag[1].imagePoint_x + arTag[2].imagePoint_x + arTag[3].imagePoint_x) / 4.0f );
            Log.i(TAG, "Center y : " + (arTag[0].imagePoint_y + arTag[1].imagePoint_y + arTag[2].imagePoint_y + arTag[3].imagePoint_y ) / 4.0f );

            Mat Center_Target = findTargetCenter_Cycle(cropPic);
            Log.i(TAG, "Targrt At =" + Center_Target.dump() );

            return  Center_Target;
        } else {
            FAIL_to_Find_TARGET = true;
            Log.i(FailTAG, "AR detectMarkers");
            Log.i(TAG, "--Fail: Only found " + ids.rows() + " markers");
            return undistortPic;
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
        return new arTag_data(AR_xDist, AR_yDist, xCenter, yCenter);
    }

    private Mat findTargetCenter_Cycle(Mat pic) {
        final String TAG = "findTargetCenter";
        Mat circles = new Mat();

        try {

                Imgproc.HoughCircles(pic, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                        (double) pic.rows() / 16, // change this value to detect circles with different distances to each other
                        100.0, 30.0, 1, 30); // change the last two parameters
                // (min_radius & max_radius) to detect larger circles
                Log.i(TAG, "Succeed HoughCircles");
                // circle center
                Mat ArucoDetectedCenter = pic.clone();

                double[] c = circles.get(0, 0);
                org.opencv.core.Point center = new org.opencv.core.Point(Math.round(c[0]), Math.round(c[1]));

                Imgproc.circle(ArucoDetectedCenter, center, 1, new Scalar(255, 0, 255), 3, 8, 0);
                api.saveMatImage(ArucoDetectedCenter, (System.currentTimeMillis() - debug_Timestart) + " ArucoDetectedCenter.png");


            double[] point = {center.x, center.y};
            Mat out = new Mat(1, 1, CvType.CV_32FC2);

            out.put(0, 0, point);
            Log.i(TAG, "Succeed findTargetCenter =" + out.dump());
            return out;

        }catch (Exception e){
            //######################
            FAIL_to_Find_TARGET = true;
            //######################
            Log.i(FailTAG, "findTargetCenter_Cycle =" + e);
            Mat out = new Mat(1, 1, CvType.CV_32FC2);
            double[] point = { -1 , -1};
            out.put(0, 0, point);

            Log.i(TAG, "Fail findTargetCenter =" + e);
            return out;
        }

    }

    private Mat undistortPic(Mat pic) {
        final String TAG = "undistortPic";
        api.saveMatImage(pic,  (System.currentTimeMillis()-debug_Timestart ) + " Input Pic to undistort.png" );
      //old
//        final double[] CAM_MATSIM = {
//                567.229305, 0.0, 659.077221,
//                0.0, 574.192915, 517.007571,
//                0.0, 0.0, 1.0
//        };
//
//        final double[] DIST_COEFFSIM = {
//                -0.216247, 0.03875, -0.010157, 0.001969, 0.0
//        };
        //new
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

            api.saveMatImage(out,  (System.currentTimeMillis()-debug_Timestart ) + " undistort pic.png");
            Log.i(TAG, "Succeed undistort  "  );
            return out;
        }catch (Exception e){
            Log.i(FailTAG, "undistortPic =" + e);
            Log.i(TAG, "Fail undistort " + e );
            return pic;
        }
    }

    private double[] pixelDistanceToAngle(double[] target ) {
        final String TAG = "pixelDistanceToAngle";
        Log.i(TAG, "================== pixelDistanceToAngle ==================");
        double[] ref  = {640, 480};
        final double xDistance = (target[0]+350) - ref[0];
        final double yDistance = ref[1] - (target[1]+350);

        final double cam2walldis = 0.54046 ; // focusCamera = 4.161542

        Log.i(TAG, "xDistance=" + xDistance);
        Log.i(TAG, "yDistance=" + yDistance);
        Log.i(TAG, "cam2walldis=" + cam2walldis);

        double k2w =  cam2walldis + cam_depth;
        double l2w =  cam2walldis - ( laser_depth - cam_depth );
        Log.i(TAG, "k2w = " + k2w);
        Log.i(TAG, "l2w = " + l2w);



        double cam_target_dis_x = ( xDistance * meter_perPx ) ;
        double cam_target_dis_y = ( yDistance * meter_perPx );
        Log.i(TAG, "cam_target_dis (x)= " + cam_target_dis_x);
        Log.i(TAG, "cam_target_dis (y)= " + cam_target_dis_y);

        //########  horizonAngle axis #######
        double horizonAngle = horizonAngle_axis(cam_target_dis_x,k2w,l2w);
        Log.i(TAG, "horizonAngle(x)= " + horizonAngle);
       //###################################

        //########  verticalAngle axis #######
        double verticalAngle = verticalAngle_axis(cam_target_dis_y,k2w,l2w);
        Log.i(TAG, "verticalAngle(y)= " + verticalAngle);
        //###################################

        double[] out = {horizonAngle, verticalAngle};
        Log.i(TAG, "Angle(x,y)= " + out);
        Log.i(TAG, "================== Done ==================");
        return out;
    }

    private double horizonAngle_axis(double cam_target_dis_x , double k2w , double l2w){
        final String TAG = "horizonAngle_axis";
        double horizonAngle = 0;

        double k2t_x = cam_target_dis_x - cam_width;
        double l2t_x = cam_target_dis_x -  ( cam_width +  laser_width );
        Log.i(TAG, "kibo 2 target _x(x)= " + k2t_x);
        Log.i(TAG, "laser 2 target _x(x)= " + l2t_x);

        double ok2t_dx = Math.sqrt((k2w * k2w) + (k2t_x * k2t_x));
        Log.i(TAG, "ok2t_dx= " + ok2t_dx);
        double ol2t_dx = Math.sqrt((l2w * l2w) + (l2t_x * l2t_x));
        Log.i(TAG, "ol2t_dx= " + ol2t_dx);
        double orl2t_dx = Math.sqrt( Math.abs((ok2t_dx * ok2t_dx) - (laser_width * laser_width)) ) - laser_depth;
        Log.i(TAG, "orl2t_dx= " + orl2t_dx);

        double cosVal_A_R = ((orl2t_dx * orl2t_dx) + (ok2t_dx * ok2t_dx) - (laser_oblique_x * laser_oblique_x)) / (2 * orl2t_dx * ok2t_dx);
        Log.i(TAG, "cosVal_A_R= " + cosVal_A_R);

        double A_R = Math.toDegrees(Math.acos(cosVal_A_R));
        Log.i(TAG, "A_R (x)= " + A_R);

        double A_K = Math.toDegrees(Math.atan(k2w / Math.abs(k2t_x) ));
        Log.i(TAG, "A_K (x)= " + A_K);

        double A_L = Math.toDegrees(Math.atan(l2w / Math.abs(l2t_x)  ));
        Log.i(TAG, "A_L (x)= " + A_L);


        if(l2t_x > 0) {

            Log.i(TAG, "target on Right of laser : " + l2t_x );
            double X_angel = A_R - (A_L - A_K);
            Log.i(TAG, "X_angel= " + X_angel);

            double Y = Math.sqrt( (ol2t_dx * ol2t_dx) + (orl2t_dx * orl2t_dx) - (2 * ol2t_dx * orl2t_dx * Math.cos(Math.toRadians(X_angel)) ));
            Log.i(TAG, "Y= " + Y);

            horizonAngle = (Math.toDegrees(Math.asin((Y / 2) / laser_oblique_x))) * 2;
        }else if(l2t_x < 0){

            Log.i(TAG, "target on Left  of laser : " + l2t_x );
            double A_angel = 180 - (A_R + A_L +  A_K);
            Log.i(TAG, "A_angel= " + A_angel);

            double X_dis = Math.sqrt((ol2t_dx * ol2t_dx) + (orl2t_dx * orl2t_dx) - 2 * ol2t_dx * orl2t_dx * Math.cos(Math.toRadians(A_angel)));
            Log.i(TAG, "X_dis= " + X_dis);

            horizonAngle = (Math.toDegrees(Math.asin( (X_dis / 2) / laser_oblique_x ))) * 2 * -1;
        }

        return  horizonAngle;
    }

    private double verticalAngle_axis(double cam_target_dis_y , double k2w , double l2w){
        final String TAG = "verticalAngle_axis";
        double verticalAngle = 0;

        double k2t_y = cam_target_dis_y + cam_high;
        double l2t_y = cam_target_dis_y - (laser_high - cam_high );
        Log.i(TAG, "kibo 2 target _y(y)= " + k2t_y);
        Log.i(TAG, "laser 2 target _y(y)= " + l2t_y);


        double ok2t_dy = Math.sqrt((k2w * k2w) + (k2t_y * k2t_y));
        Log.i(TAG, "ok2t_dy= " + ok2t_dy);
        double ol2t_dy = Math.sqrt((l2w * l2w) + (l2t_y * l2t_y));
        Log.i(TAG, "ol2t_dy= " + ol2t_dy);
        double orl2t_dy =  Math.sqrt( Math.abs((ok2t_dy * ok2t_dy) - (laser_high * laser_high )) )  - laser_depth;
        Log.i(TAG, "orl2t_dy= " + orl2t_dy);

        double cosVal_A_R = ((orl2t_dy * orl2t_dy) + (ok2t_dy * ok2t_dy) - (laser_oblique_y * laser_oblique_y )) / (2 * orl2t_dy * ok2t_dy);
        Log.i(TAG, "cosVal_A_R= " + cosVal_A_R);

        double A_R = Math.toDegrees(Math.acos(cosVal_A_R));
        Log.i(TAG, "A_R (y)= " + A_R);

        double A_K = Math.toDegrees(Math.atan(k2w / Math.abs(k2t_y) ));
        Log.i(TAG, "A_K (y)= " + A_K);

        double A_L = Math.toDegrees(Math.atan(l2w / Math.abs(l2t_y)  ));
        Log.i(TAG, "A_L (y)= " + A_L);

        double A_angel =  A_K - (A_R + A_L);
        Log.i(TAG, "X_angel= " + A_angel);

        double X_dis = Math.sqrt((ol2t_dy * ol2t_dy) + (orl2t_dy * orl2t_dy) - 2 * ol2t_dy * orl2t_dy * Math.cos(Math.toRadians(A_angel)));
        Log.i(TAG, "X_dis= " + X_dis);

        verticalAngle = (Math.toDegrees(Math.asin( (X_dis / 2) / laser_oblique_x ))) * 2 * -1;

        return  verticalAngle;
    }
    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
        final String TAG = "Convert euler angle to quaternion";

        xAngle = Math.toRadians(xAngle);
        yAngle = Math.toRadians(yAngle);
        zAngle = Math.toRadians(zAngle);
        double c1 = Math.cos(yAngle/2);
        double c2 = Math.cos(zAngle/2);
        double c3 = Math.cos(xAngle/2);
        double s1 = Math.sin(yAngle/2);
        double s2 = Math.sin(zAngle/2);
        double s3 = Math.sin(xAngle/2);

        double w = c1*c2*c3 - s1*s2*s3;
        double x = s1*s2*c3 + c1*c2*s3;
        double y = s1*c2*c3 + c1*s2*s3;
        double z = c1*s2*c3 - s1*c2*s3;

        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float) y, (float)z, (float)w);
    }

    private Quaternion combineQuaternion(Quaternion newOrientation, Quaternion oldOrientation) {
        String TAG = "combineQuaternion";
        double x =  newOrientation.getX() * oldOrientation.getW() + newOrientation.getY() * oldOrientation.getZ()
                - newOrientation.getZ() * oldOrientation.getY() + newOrientation.getW() * oldOrientation.getX();
        double y = -newOrientation.getX() * oldOrientation.getZ() + newOrientation.getY() * oldOrientation.getW()
                + newOrientation.getZ() * oldOrientation.getX() + newOrientation.getW() * oldOrientation.getY();
        double z =  newOrientation.getX() * oldOrientation.getY() - newOrientation.getY() * oldOrientation.getX()
                + newOrientation.getZ() * oldOrientation.getW() + newOrientation.getW() * oldOrientation.getZ();
        double w = -newOrientation.getX() * oldOrientation.getX() - newOrientation.getY() * oldOrientation.getY()
                - newOrientation.getZ() * oldOrientation.getZ() + newOrientation.getW() * oldOrientation.getW();
        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }
}

