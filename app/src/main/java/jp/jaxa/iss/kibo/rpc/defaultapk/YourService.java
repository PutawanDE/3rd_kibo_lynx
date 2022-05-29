package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.core.Mat;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.lang.*;
import java.util.Arrays;

import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.undistort;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 4;
    boolean check_point_A = false;
    boolean check_target_1 = false;
    boolean check_point_B = false;
    boolean check_target_2 = false;

    boolean It_realworld = false;
    final String FailTAG = "Fail_Service";

    boolean debug = true;
    long debug_Timestart = 0;

    final Point point_A_to_shoot_target1 = new Point(10.71, -7.811971 , 4.48);
    final Point point_B_to_shoot_target2 = new Point(11.2746, -9.92284, 5.29881); // original
    final Point point_Goal_target = new Point(11.2746, -7.89178, 4.96538);

    final Quaternion B_quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);





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

        mission_report();
    }

    @Override
    protected void runPlan2(){}
    @Override
    protected void runPlan3(){}

    private boolean mission_point_A(){
        final Point point_A = new Point(10.71000, -7.70000, 4.48000);
        final Quaternion q_A = new Quaternion(0.0f, 0.707f, 0.0f, 0.707f);
        boolean point_A_Succeeded;

        try {
            point_A_Succeeded = move_to(point_A, q_A);
            api.reportPoint1Arrival();

            Bitmap image1_A = api.getBitmapNavCam();
            api.saveBitmapImage(image1_A, (System.currentTimeMillis()-debug_Timestart )+"Point A Arrival.png");
            return point_A_Succeeded;
        }catch (Exception e){
            Log.i(FailTAG, "mission_point_A =" + e);
            return  false;
        }
    }

    private boolean mission_target_1(){
         try {
             boolean aim_Succeed;

             // AI aim and shoot
             int aim_try = 0;
             do {
                 Quaternion q_target1 = new Quaternion(0f, 0.707f, 0f, 0.707f);
                 aim_Succeed = move_to(point_A_to_shoot_target1, q_target1);
                 aim_try++;
             }while (!aim_Succeed && aim_try < LOOP_MAX );

             //############## SAFETY ####################
             if(check_point_A && It_realworld){
                 return false;
             }
             //##################################

             if(debug){
                 aim_Succeed = true;
             }

            if(aim_Succeed) {
                api.laserControl(true);
                api.takeTarget1Snapshot();
//                Bitmap laserPic = api.getBitmapNavCam();
//                api.saveBitmapImage(laserPic, (System.currentTimeMillis()-debug_Timestart )+"laser targer 1 at A.png");
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
            api.saveBitmapImage(image1_B, (System.currentTimeMillis()-debug_Timestart )+"Point B Arrival.png");

            return  point_B_prime_Succeeded;

        }catch (Exception e){
            Log.i(FailTAG, "mission_point_B =" + e);
            return  false;
        }
    }

    private boolean mission_target_2(){
        final String TAG = "mission_target_2";
        try {
            boolean aim_Succeed;
            boolean on_Laser = false;
            int aim_try = 0;

            do {// AI aim and shoot
                aim_Succeed = aim_shoot();
                aim_try++;
            }while (!aim_Succeed && aim_try < LOOP_MAX );
            Log.i(TAG, "aim status = "  +" :" + aim_Succeed);

            //############## SAFETY ####################
            if(NavCam.FailtoFindTarget() && It_realworld){
                return false;
            }
            //##################################

            if(debug){
                aim_Succeed = true;
            }
            if(aim_Succeed) {


                api.laserControl(true);
                api.takeTarget2Snapshot();
                api.laserControl(false);
                return  true;

            }else {
                Log.i(FailTAG, "FAIL to_shoot target_2");
                return false;
            }
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
            for (Boolean aBoolean : Mission_State) {
                Log.i(aBoolean.toString(), "Mission State");
            }
        }

        do {
            cleared = api.reportMissionCompletion();
        } while (!cleared);

    }

    private boolean move_to(Point target_point , Quaternion q) {
        final String TAG = "move_to";

        int counter = 0;
        Result result;

        Log.i(TAG, "Start move to p:"  +target_point +", q:" + q);

        do {
            result = api.moveTo(target_point, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        Log.i(TAG, "Done move to p:"  +target_point +", q:" + q);
        Log.i(TAG, "Api position:"  + api.getRobotKinematics().getPosition() +", q:" + api.getRobotKinematics().getOrientation());
        return  result.hasSucceeded();
    }

//    private void turn_to(Quaternion q) {
//        final String TAG = "turn_to";
//        final Point p_dormant = new Point(0, 0, 0);
//
//        int counter = 0;
//        Result result;
//
//        Log.i(TAG, "Start turn to p:"  + p_dormant +", q:" + q);
//
//        do {
//            result = api.relativeMoveTo(p_dormant, q, true);
//            counter++;
//        } while (!result.hasSucceeded() && (counter < LOOP_MAX));
//
//        Log.i(TAG, "Done turn to p:"  +p_dormant +", q:" + q);
//        Log.i(TAG, "Api position:"  + api.getRobotKinematics().getPosition() +", q:" + api.getRobotKinematics().getOrientation());
//
//    }

    private boolean  aim_shoot(){
        final String TAG = "aim_shoot";
        boolean enable_aim = true;
        try {
                Mat buffer =  api.getMatNavCam();
                api.saveMatImage(buffer,  (System.currentTimeMillis()-debug_Timestart )+" Point B buffer.png" );

                Thread.sleep(6000);
                Mat pic_cam =  api.getMatNavCam();
                Thread.sleep(1000);

                NavCam camArTag = new NavCam();
                Mat undistort_Cam = camArTag.undistortPicture(pic_cam);
                Mat  undistort_AR_Center = camArTag.findARtag(undistort_Cam);

                api.saveMatImage(camArTag.getCenterTarget(),  (System.currentTimeMillis()-debug_Timestart )+" ArucoDetectedCenter.png" );

                double[] targetPosition = undistort_AR_Center.get(0, 0);

                Aimer aimer = new Aimer(NavCam.getArTag_sizePx() , NavCam.getMeter_perPx() );
                double[] angleToTurn = aimer.pixelDistanceToAngle(targetPosition);

                if(enable_aim) {  // turn or move kibo to aim
                    Quaternion relativeQ  = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
                    Quaternion qToTurn_Target2 = combineQuaternion(relativeQ , B_quaternion);
                    Log.i(TAG, "relativeQ = " + relativeQ.toString());
                    Log.i(TAG, "qToTurn_Target2 = " + qToTurn_Target2.toString());

                    // try rotating to wanted angle and shoot laser
                    int retry = 0;
                    double angleDiffOld = 90;
                    while (retry <= LOOP_MAX) {
                        move_to(point_B_to_shoot_target2 , qToTurn_Target2); // rotate astrobee

                        // compare result with what we need
                        Kinematics res = api.getRobotKinematics();
                        double angleDiff = compareQuaternion(qToTurn_Target2, res.getOrientation());

                        Log.i(TAG, String.format("angleDiff = %f", angleDiff));
                        Log.i(TAG, String.format("Result %d = %s, %s, %s", retry, res.getConfidence(), res.getPosition().toString(), res.getOrientation().toString()));

                        if (angleDiff < 0.4) {
                            try {
                                Log.i(TAG,"angleDiff < 0.4");
                                Thread.sleep(500);
                            } catch (Exception ignored) {}
                            break;
                        }
                        else if( angleDiffOld < 1 && angleDiff < angleDiffOld ){
                            try {
                                Log.i(TAG,"angleDiff better");
                                Thread.sleep(500);
                            } catch (Exception ignored) {}
                            break;
                        }
                        else if (retry == 4) {
                            try {
                                Log.i(TAG,"retry == 4 out of bound");
                                Thread.sleep(500);
                            } catch (Exception ignored) {}
                            break;
                        }
                        angleDiffOld = angleDiff;
                        move_to(point_B_to_shoot_target2, B_quaternion); // reset astrobee
                        retry++;
                    }
                }
                return  true;
        }catch (Exception e){
            Log.i(FailTAG, "aim_shoot =" + e);
            Log.i(TAG, "Fail aim_shoot =" + e);
            return  false;
        }
    }

    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
        final String TAG = "Convert euler angle to quaternion";

        double c1 = Math.cos(yAngle / 2);
        double c2 = Math.cos(zAngle / 2);
        double c3 = Math.cos(xAngle / 2);
        double s1 = Math.sin(yAngle / 2);
        double s2 = Math.sin(zAngle / 2);
        double s3 = Math.sin(xAngle / 2);

        double w = c1 * c2 * c3 - s1 * s2 * s3;
        double x = s1 * s2 * c3 + c1 * c2 * s3;
        double y = s1 * c2 * c3 + c1 * s2 * s3;
        double z = c1 * s2 * c3 - s1 * c2 * s3;

        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float) x, (float) y, (float) z, (float) w);
    }

    private Quaternion combineQuaternion(Quaternion q1, Quaternion q2) {
        String TAG = "combineQuaternion";
        double s1 = q1.getW();
        Vector3 v1 = new Vector3(q1.getX(), q1.getY(), q1.getZ());
        double s2 = q2.getW();
        Vector3 v2 = new Vector3(q2.getX(), q2.getY(), q2.getZ());

        Vector3 a = v2.scale(s1).sum(v1.scale(s2));
        Vector3 b = v1.cross(v2);
        Vector3 res = a.sum(b);
        double w = s1*s2 - v1.dot(v2);
        return new Quaternion((float) res.getX(), (float) res.getY(), (float) res.getZ(), (float) w);
    }

    private Vector3 vectorOrientation(Quaternion q) {
        Vector3 v = new Vector3(1, 0, 0); // (0, 0, 0, 1) quaternion -> vector
        Vector3 u = new Vector3(q.getX(), q.getY(), q.getZ());
        double s = q.getW();

        Vector3 a =  u.scale(2.0 * u.dot(v));
        Vector3 b = v.scale(s*s - u.dot(u));
        Vector3 c = u.cross(v).scale(2.0 * s);
        return a.sum(b).sum(c);
    }
    private double vectorAngleDiff(Vector3 v1, Vector3 v2) {
        double a = v1.dot(v2);
        double b = v1.size() * v2.size();
        return Math.toDegrees(Math.acos(a/b));
    }
    private double compareQuaternion(Quaternion q1, Quaternion q2) {
        return vectorAngleDiff(vectorOrientation(q1), vectorOrientation(q2));
    }
}

