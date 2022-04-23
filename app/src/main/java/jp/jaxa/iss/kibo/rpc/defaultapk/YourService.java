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

    boolean FAIL_to_READ_AR= false;
    final String FailTAG = "Fail_method";


    boolean debug = true;
    boolean debug_pointB_unaim_snap = false;
    boolean debug_pointB_aim_snap = false;
    long debug_Timestart = 0;

    double arTag_sizePx= -1;
    double meter_perPx= -1;

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

        // shoot point 2
        check_target_2 = mission_target_2();

//        mission_test();
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

            final Point point_50cm = new Point(10.71000, -7.5000, 4.48000);
            final Quaternion q_A = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);
            check_point_A = mission_point_A();

            api.laserControl(true);
            api.takeTarget1Snapshot();
            Thread.sleep(5000);
            Bitmap laserPic = api.getBitmapNavCam();
            api.saveBitmapImage(laserPic, "laser targer 1 at A");
            api.laserControl(false);

            Thread.sleep(10000);


            move_to(point_50cm, q_A);
            api.laserControl(true);
            api.takeTarget1Snapshot();
            Thread.sleep(5000);
            Bitmap Pic_50 = api.getBitmapNavCam();
            api.saveBitmapImage(Pic_50, "laser targer 2 at A");
            api.laserControl(false);

            Thread.sleep(5000);


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

            //       save the image
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


            if(debug){
                aim_Succeed = true;
            }

            if(aim_Succeed) {
                //#######################
                if(FAIL_to_READ_AR && !debug){
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
        final Point point_B_target = new Point(11.2746, -9.92284, 5.29881);

        move_to(point_B_0,q_B);
        move_to(point_B_1,q_B);
        move_to(point_B_target,q_B);

        boolean point_B_prime_Succeeded;
        try {
            move_to(point_B_0,q_B);
            move_to(point_B_1,q_B);
            point_B_prime_Succeeded = move_to(point_B_target,  q_B);

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
                if(i == 0 || i == 5){
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
                    if(FAIL_to_READ_AR && !debug){
                        Log.i(FailTAG, "FAIL_to_READ_AR target_2");
                        return  false;
                    }
                    //#######################
                    if(!on_Laser){
                        api.laserControl(true);
                        on_Laser = true;
                    }
                    api.takeTarget2Snapshot();


                    if(i == 0 || i == 5){
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
        final Point point_Goal_2 = new Point(11.2746, -7.89178, 4.96538);

        boolean cleared;

        move_to(point_Goal_0,q_Goal);
        move_to(point_Goal_1,q_Goal);
        move_to(point_Goal_2,q_Goal);

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

        Log.i(TAG, "Start");

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        //log_kinematics();
        Log.i(TAG, "Done");
        return  result.hasSucceeded();
    }

    private boolean  aim_shoot(String targetType){

            Mat undistort_Cam = undistortPic(api.getMatNavCam());

            if (targetType.equals("A")){
//                api.saveMatImage(undistort_Cam, "Point A unaim");

                Quaternion q_target1 = new Quaternion(0f, 0.707f, 0f, 0.707f);
                Point point_to_shoot_target1 = new Point(10.71, -7.811971 , 4.48);
                move_to(point_to_shoot_target1, q_target1);


//                Mat point_A_aim = undistortPic(api.getMatNavCam());
//                api.saveMatImage(point_A_aim, "A Point A aim");


            }else if(targetType.equals("B")){

                if(debug && !debug_pointB_unaim_snap){
                    api.saveMatImage(undistort_Cam, "Point B unaim" + ( debug_Timestart - System.currentTimeMillis() ));
                    debug_pointB_unaim_snap = true;
                }
                final Point point_B_target = new Point(11.2746, -9.92284, 5.29881);

//              double[] laser = {711, 455};

                //#######################################
                // FIX Quaternion to shoot target2
                Mat  undistort_AR_Center = cam_ar_read(undistort_Cam);
//                double[] angleToTurn = pixelDistanceToAngle(undistort_AR_Center.get(0, 0));
//                Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
//                Quaternion qToTurn_Target2  = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
//                move_to(point_B_target, qToTurn_Target2);
                //#######################################

                if(debug && !debug_pointB_aim_snap){
                    Mat undistort_B_Aim = undistortPic(api.getMatNavCam());
                    api.saveMatImage(undistort_B_Aim, "Point B aim" + ( debug_Timestart - System.currentTimeMillis() ));
                    debug_pointB_aim_snap = true;
                }



            }else {
                Log.i(FailTAG, "aim_shoot mode Fail");
            }
        return  true;
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


    private Mat cam_ar_read(Mat undistortPic) {
        final String TAG = "ar_read";
        final long start = System.currentTimeMillis();
        final Dictionary bluePrint = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
                long end = System.currentTimeMillis();
//                Log.i(TAG, "sleep_times=" + (end-start));
                Log.i(TAG, "Reading AR");

        int counter = 0;
        while (ids.rows() < 4 && counter < LOOP_MAX) { // try 3 until find all 4 ids
            //############ detect Markers ############
            Mat pic = new Mat(undistortPic, new Rect(320, 240, 640, 500));
            Aruco.detectMarkers(pic, bluePrint, corners, ids); // find all ids
            //########################
            counter++;
        }
        Log.i(TAG, "ids= " + ids.dump());
                end = System.currentTimeMillis();
                Log.i(TAG, "ar_read_time=" + (end-start));

        arTag_data[] arTag = new  arTag_data[4];

        if(ids.rows() == 4) {
            Log.i(TAG, "All 4 ids are found.");

            double ar_x_avg_px=0;
            double ar_y_avg_px=0;


            for (int i = 0; i < 4; i++) {
                arTag[i] =  getArTag_data(corners.get(i));
                //                Log.i(TAG, "Marker Center[" + i + "](id: " + ids.get(i, 0)[0] + ")=" + markersCenter[i].dump());

                Log.i(TAG, "imagePoint_x at "+ i +":"+ + (int)arTag[i].imagePoint_x + ",imagePoint_y at " + i +":"+ (int) arTag[i].imagePoint_y  );

                ar_x_avg_px += arTag[i].size_x;
                ar_y_avg_px += arTag[i].size_y;
            }
            arTag_sizePx = (ar_x_avg_px/4 + ar_y_avg_px/4)  / 2  ;
            meter_perPx = 0.05 / arTag_sizePx;

            Log.i(TAG, "arTag_sizePx : " + arTag_sizePx );
            Log.i(TAG, "meter_perPx : " + meter_perPx );
            Log.i(TAG, "Center x : " + (arTag[0].imagePoint_x + arTag[1].imagePoint_x + arTag[2].imagePoint_x + arTag[3].imagePoint_x) / 4.0f );
            Log.i(TAG, "Center y : " + (arTag[0].imagePoint_y + arTag[1].imagePoint_y + arTag[2].imagePoint_y + arTag[3].imagePoint_y ) / 4.0f );
            int[] rect = findRect_crop(arTag);

            int start_x = rect[0];
            int start_y = rect[1];
            int pic_width =  rect[2];
            int pic_height = rect[3] ;

            Log.i(TAG, "Rect" + start_x + "," + (int) start_y +"," + pic_width   +"," + pic_height );

            Mat crop_findCenter = new Mat(undistortPic, new Rect(start_x, start_y,pic_width, pic_height));
            api.saveMatImage(crop_findCenter, "Point B crop target");

            Mat Center_Target = findTargetCenter_Cycle(crop_findCenter);
//        Log.i(TAG, "distorted=" + AR_Center.dump());

//        end = System.currentTimeMillis();
//        Log.i(TAG, "ar_read+process_time=" + (end-start));
            return  Center_Target;
        } else {
            FAIL_to_READ_AR = true;
            Log.i(FailTAG, "AR detectMarkers");
             Log.i(TAG, "--Fail: Only found " + ids.rows() + " markers");
            return undistortPic;
        }
    }
    private int[] findRect_crop(arTag_data[] arTag){
        int min_x = Integer.MAX_VALUE;
        int max_x = -1;
        int min_y = Integer.MAX_VALUE;
        int max_y = -1;

        for (int i = 0; i < 4; i++) {
            if (arTag[i].imagePoint_x < min_x) {
                min_x = (int) arTag[i].imagePoint_x;
            }
            if (arTag[0].imagePoint_x > max_x) {
                max_x = (int) arTag[i].imagePoint_x;
            }
            if (arTag[i].imagePoint_y < min_y) {
                min_y = (int) arTag[i].imagePoint_y;
            }
            if (arTag[i].imagePoint_y > max_y) {
                max_y = (int) arTag[i].imagePoint_y;
            }
        }
        int[] rect = {min_x,max_y,max_x - min_x,max_y - min_y};
        return rect;
        }

    private arTag_data getArTag_data (Mat corners) { // find center && distance
        double xCenter;
        double yCenter;
        double xLeft=0;
        double xRight=0;
        double yUp=0;
        double yDown=0;


        double AR_xDist=0;
        double AR_yDist=0;
        xCenter = (corners.get(0, 0)[0] + corners.get(0, 1)[0] + corners.get(0, 2)[0] + corners.get(0, 3)[0]) / 4.0f;

        yCenter = (corners.get(0, 0)[1] + corners.get(0, 1)[1] + corners.get(0, 2)[1] + corners.get(0, 3)[1]) / 4.0f;


        // ###########################
        // find in undistort ???
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
        AR_yDist = (yUp-yDown)/2;
        // ###########################
        return new arTag_data(AR_xDist,AR_yDist,xCenter,yCenter);
    }
    private Mat findTargetCenter_Cycle(Mat pic) {

        // riw code
        // find cycle center only in area p1 p2 p3 p4
        //        float xCenter = (p1.x + p2.x + p3.x + p4.x) / 4.0f;
        //        float yCenter = (p1.y + p2.y + p3.y + p4.y) / 4.0f;
        //
        final String TAG = "findTargetCenter";
        Mat circles = new Mat();
//        Mat gray = new Mat();
//        Imgproc.cvtColor(pic, gray, Imgproc.COLOR_BGR2GRAY);
//        Imgproc.medianBlur(pic, gray, 5);

        try {
            Imgproc.HoughCircles(pic, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                    (double)pic.rows()/16, // change this value to detect circles with different distances to each other
                    100.0, 30.0, 1, 30); // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
            Log.i(TAG, "Succeed HoughCircles");
            // circle center
            double[] c = circles.get(0, 0);
            org.opencv.core.Point center = new org.opencv.core.Point(Math.round(c[0]), Math.round(c[1]));

            Mat out = new Mat(1, 1, CvType.CV_32FC2);
            double[] point = {center.x, center.y};
            out.put(0, 0, point);
            Log.i(TAG, "Succeed findTargetCenter =" + out.dump());
            api.saveMatImage(out, "Point B target center");
            return out;

        }catch (Exception e){
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
        api.saveMatImage(pic, "Input Pic to undistort");
        final double[] CAM_MATSIM = {
                567.229305, 0.0, 659.077221,
                0.0, 574.192915, 517.007571,
                0.0, 0.0, 1.0
        };

        final double[] DIST_COEFFSIM = {
                -0.216247, 0.03875, -0.010157, 0.001969, 0.0
        };
        // in -> rows:1, cols:4
        // in -> 1xN 2 Channel
        Log.i(TAG, "Start");

        try {
            Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
            Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

            cameraMat.put(0, 0, CAM_MATSIM);
            distCoeffs.put(0, 0, DIST_COEFFSIM);
        Mat out = new Mat(pic.rows(), pic.cols(), pic.type());

//            Mat out = new Mat();

            undistort(pic, out, cameraMat,distCoeffs);

            api.saveMatImage(out, "undistort pic");
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
        double[] ref  = {711, 455};
        final double xDistance = target[0] - ref[0];
        final double yDistance = ref[1] - target[1];

        // f = 0.6233 * 41.5 / 5 = 5.17339

        final double focusCamera = 5.17339;
        //final double anglePerPixel = 130 / Math.sqrt(Math.pow(NAV_MAX_WIDTH, 2) + Math.pow(NAV_MAX_HEIGHT, 2));




        final double anglePerPixel = 0.08125;
        Log.i(TAG, "xDistance=" + xDistance);
        Log.i(TAG, "yDistance=" + yDistance);
        Log.i(TAG, "anglePerPixel=" + anglePerPixel);

        double xAngle = xDistance * anglePerPixel;
        double yAngle = yDistance * anglePerPixel;

        Log.i(TAG, "xAngle=" + xAngle);
        Log.i(TAG, "yAngle=" + yAngle);

        double[] out = {xAngle, yAngle};
        return out;
    }

    private  double getkibo_distanceformwall (){

        return  0;
    }

    private double find_horizon_angle(double d_h_laser_center , double  d_h_kibo_center){
        double d_laser_center = d_h_laser_center;
        double d_kibo_center = d_h_kibo_center;
        double deg_h = toDegrees(atan(d_laser_center/d_kibo_center));
        return deg_h;
    }
    private double find_vertical_angle(double d_v_laser_center , double  d_v_kibo_center){
        double d_laser_center = d_v_laser_center;
        double d_kibo_center = d_v_kibo_center;
        double horizon_angel = atan(d_laser_center/d_kibo_center);

        return horizon_angel;
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

