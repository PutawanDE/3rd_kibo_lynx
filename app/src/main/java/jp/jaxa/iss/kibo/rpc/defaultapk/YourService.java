package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
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


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 5;
    boolean check_point_A = false;
    boolean check_target_1 = false;
    boolean check_point_A_prime = false;
    boolean check_target_2 = false;

    boolean check_report = false;

    boolean debug = false;


    double AR_xDist=0;
    double AR_yDist=0;

    @Override
    protected void runPlan1(){
        api.startMission();


        mission_testmove();
        // move to point A
//        check_point_A = mission_point_A();

        // shoot point 1

//        check_target_1 = mission_target_1();

        // move to point A'

//        check_point_A_prime = mission_point_A_prime();

        // shoot point 1
//        check_target_2 = mission_target_2();

        if(debug){
            Boolean[] Mission_State = new  Boolean[]{check_point_A,check_target_1,check_point_A_prime,check_target_2};
            for (int i = 0; i < Mission_State.length; i++) {
                Log.i(Mission_State[i].toString(), "Mission State");
            }
        }


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




    private void mission_testmove(){
        final Point point_A = new Point(10.71000, -7.70000, 4.48000);
        final Quaternion q_A = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);



        final Point point_Goal = new Point(10.71000, -7.70000, 4.48000);
        final Quaternion q_Goal = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);





        try {
             move_to(point_A, q_A);

            api.reportPoint1Arrival();

            api.laserControl(true);
            api.takeTarget1Snapshot();
            api.laserControl(false);
            Bitmap image_A = api.getBitmapDockCam();
            Bitmap image1_A = api.getBitmapNavCam();
//             save the image
            api.saveBitmapImage(image_A, "A DockCam");
            api.saveBitmapImage(image1_A, "A NavCam");

            Thread.sleep(1000);
            //move to b
            move_to_B();

            api.laserControl(true);
            api.takeTarget1Snapshot();
            api.laserControl(false);


            Bitmap image_B = api.getBitmapDockCam();
            Bitmap image1_B = api.getBitmapNavCam();
//             save the image
            api.saveBitmapImage(image_B, "B DockCam");
            api.saveBitmapImage(image1_B, "B NavCam");

            //move to goal
            move_to_Goal();


        }catch (Exception e){

        }
    }




    private boolean mission_point_A(){
        final Point point_A = new Point(10.71000, -7.70000, 4.48000);
        final Quaternion q_A = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);
        boolean point_A_Succeeded;

        boolean qr_read_Succeeded;

        try {
            point_A_Succeeded = move_to(point_A, q_A);
            api.reportPoint1Arrival();
            return point_A_Succeeded;
        }catch (Exception e){
            return  false;
        }
    }


    private boolean mission_target_1(){
         try {
             boolean aim_Succeed;

             // AI aim and shoot
             int aim_try = 0;
             do {
                 aim_Succeed = aim_shoot();
                 aim_try++;
             }while (!aim_Succeed && aim_try < LOOP_MAX );

            if(aim_Succeed) {
                api.laserControl(true);
                api.takeTarget1Snapshot();
                api.laserControl(false);
            }
//             }
             return  true;
         }catch (Exception e){
             return  false;
         }

    }


    private boolean mission_point_A_prime(){
        final Point point_A_prime = new Point(11.27460, -9.92284, 5.29881);
        final Quaternion q_A_prime = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);
        boolean point_A_prime_Succeeded;
        try {
            point_A_prime_Succeeded = move_to(point_A_prime,  q_A_prime);
            return  point_A_prime_Succeeded;

        }catch (Exception e){
            return  false;
        }
    }

    private boolean mission_target_2(){
        try {
            boolean aim_Succeed;
            api.laserControl(true);
            for (int shoot = 0 ; shoot < 10 ; shoot++){

                // AI aim and shoot
                int aim_try = 0;
                do {
                    aim_Succeed = aim_shoot();
                    aim_try++;
                }while (!aim_Succeed && aim_try < LOOP_MAX );

                if(aim_Succeed) {
                    api.takeTarget2Snapshot();
                }

            }
            api.laserControl(false);
            return  true;
        }catch (Exception e){
            return  false;
        }
    }

    private void mission_report(){
        final Point point_report = new Point(11.27460, -9.92284, 5.29881);
        final Quaternion q_report = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);

        boolean cleared;

        move_to(point_report,  q_report);





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

    private boolean move_to_A(){

        return  true;
    }
    private boolean move_to_B(){
        final Quaternion q_B = new Quaternion(0f, 0f, -0.707f, 0.707f);

        final Point point_B_0 = new Point(10.876214285714285, -8.5, 4.97063);
        final Point point_B_1 = new Point(11.0067, -9.44819, 5.186407142857143);
        final Point point_B_2 = new Point(11.2746, -9.92284, 5.29881);


        move_to(point_B_0,q_B);
        move_to(point_B_1,q_B);
        move_to(point_B_2,q_B);

        return true;
    }
    private  boolean move_to_Goal(){
        final Quaternion q_Goal = new Quaternion(0f, 0f, -0.707f, 0.707f);

        final Point point_Goal_0 = new Point(11.0067, -9.44819, 5.1722);
        final Point point_Goal_1 = new Point(11.073773469387755, -8.5, 4.97063);
        final Point point_Goal_2 = new Point(11.2746, -7.89178, 4.96538);

        move_to(point_Goal_0,q_Goal);
        move_to(point_Goal_1,q_Goal);
        move_to(point_Goal_2,q_Goal);

        return true;
    }

    class imagePoint {
        float x, y;
        imagePoint(float x, float y) {
            this.x = x;
            this.y = y;
        }

        String dump() {
            return ("[" + x + ", " + y + "]");
        }
    }

    private boolean  aim_shoot(){

          Mat AR_Center = ar_read();

          Mat undistortAr = undistortPoints(AR_Center);
          double[] laser = {711, 455};


         double[] angleToTurn = pixelDistanceToAngle(undistortAr.get(0, 0));
         Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);

         Quaternion qToTurn  = combineQuaternion(imageQ, api.getRobotKinematics().getOrientation());

         return  true;

    }
    private Mat ar_read() {
        final String TAG = "ar_read";
//        final long start = System.currentTimeMillis();
        final Dictionary bluePrint = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
//        long end = System.currentTimeMillis();
//        Log.i(TAG, "sleep_times=" + (end-start));
        //        Log.i(TAG, "Reading AR");

        int counter = 0;
        while (ids.rows() < 4 && counter < LOOP_MAX) {

            //############ detect Markers ############
            Mat pic = new Mat(api.getMatNavCam(), new Rect(320, 240, 640, 500));
            Aruco.detectMarkers(pic, bluePrint, corners, ids);
            //########################
            counter++;
        }


        // for debug
            Log.i("corners", "corners size" + corners.size() );

        for (int j = 0; j < corners.size(); j++) {
            Mat temp = corners.get(j);

            for (int z = 0; z < temp.cols(); z++) {
                double[] t = temp.get(0, z);
                t[0] = t[0] + 320;
                t[1] = t[1] + 240;
                temp.put(0, z, t);
            }

            Log.i("corners", "corners[" + j + "]=" + temp.dump());
        }

        Log.i(TAG, "ids= " + ids.dump());
//        end = System.currentTimeMillis();
//        Log.i(TAG, "ar_read_time=" + (end-start));
        //

        imagePoint[] markersCenter = new imagePoint[4];

        if(ids.rows() == 4) {
//            Log.i(TAG, "All 4 ids are found.");
            for (int i = 0; i < 4; i++) {
                markersCenter[i] = findCenterAR(corners.get(i));
//                Log.i(TAG, "Marker Center[" + i + "](id: " + ids.get(i, 0)[0] + ")=" + markersCenter[i].dump());
            }

        } else {
//            Log.i(TAG, "--Fail: Only found " + ids.rows() + " markers");
        }

        final Mat Center_AR = findTargetCenterRect(markersCenter[0], markersCenter[1], markersCenter[2], markersCenter[3]);
//        Log.i(TAG, "distorted=" + AR_Center.dump());

//        end = System.currentTimeMillis();
//        Log.i(TAG, "ar_read+process_time=" + (end-start));
        return  Center_AR;
    }


    private imagePoint findCenterAR(Mat corners) {
        double xCenter;
        double yCenter;
        double xLeft=0;
        double xRight=0;
        double yUp=0;
        double yDown=0;



        xCenter = (corners.get(0, 0)[0] + corners.get(0, 1)[0] + corners.get(0, 2)[0] + corners.get(0, 3)[0]) / 4.0f;

        yCenter = (corners.get(0, 0)[1] + corners.get(0, 1)[1] + corners.get(0, 2)[1] + corners.get(0, 3)[1]) / 4.0f;

        //for find left right down up
        for(int i =0 ; i<4 ; i++){
            if(corners.get(0, i)[0]<xCenter){
                xLeft +=corners.get(0, i)[0];
            }else
                xLeft += corners.get(0, i)[0];

        }
        for (int i = 0 ; i<4 ; i++){
            if(corners.get(0, i)[1]<yCenter){
                yDown+=corners.get(0, i)[1];
            }else {
                yUp+=corners.get(0, i)[1];
            }
    }
    AR_xDist = (xRight-xLeft)/2;
    AR_yDist = (yUp-yDown)/2;

        return new imagePoint((float)xCenter, (float)yCenter);
    }
    private Mat findTargetCenterRect(imagePoint p1, imagePoint p2, imagePoint p3, imagePoint p4) { // riw code
        float xCenter = (p1.x + p2.x + p3.x + p4.x) / 4.0f;
        float yCenter = (p1.y + p2.y + p3.y + p4.y) / 4.0f;

        Mat out = new Mat(1, 1, CvType.CV_32FC2);
// find cycle center only in area p1 p2 p3 p4

        float[] point = {xCenter, yCenter};
        out.put(0, 0, point);

        return out;
    }


    private double[] pixelDistanceToAngle(double[] target ) {
        final String TAG = "pixelDistanceToAngle";
        double[] ref  = {711, 455};
        final double xDistance = target[0] - ref[0];
        final double yDistance = ref[1] - target[1];
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



    private Mat undistortPoints(Mat points) {
        final String TAG = "undistortCorner";
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

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        Mat out = new Mat(points.rows(), points.cols(), points.type());

        Imgproc.undistortPoints(points, out, cameraMat, distCoeffs, new Mat(), cameraMat);

        Log.i(TAG, "undistort=" + out.dump());
        // out -> 1xN 2 Channel
        return out;
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

