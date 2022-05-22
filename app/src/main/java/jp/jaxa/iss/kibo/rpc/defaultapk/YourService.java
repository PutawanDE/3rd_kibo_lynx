package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 3;

    final double[] CAM_MATSIM = {
            523.105750, 0.000000, 635.434258,
            0.000000, 534.765913, 500.335102,
            0.000000, 0.000000, 1.000000
    };


    final double[] DIST_COEFFSIM = {
            -0.164787, 0.020375, -0.001572, -0.000369, 0.000000
    };

    private void move_to(Point p, Quaternion q) {
        int counter = 0;
        Result result;
        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));
    }

    private void move_to(double x, double y, double z, Quaternion q) {
        final Point p = new Point(x, y, z);
        move_to(p, q);
    }

    private Mat undistortCroppedImg(Mat img, int x, int y) {
        // change principal point of CAM_MATSIM
        final double[] CAM_MAT = CAM_MATSIM;
        CAM_MAT[2] -= x;
        CAM_MAT[5] -= y;

        Mat ud_img = new Mat(img.rows(), img.cols(), img.type());
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MAT);
        distCoeffs.put(0, 0, DIST_COEFFSIM);
        Imgproc.undistort(img, ud_img, cameraMat, distCoeffs);

        return ud_img;
    }

    // down angle
    private double computeXAngle(double l2t, double yc) {
        final double navX = 0.1302;
        final double navY = 0.1111;

        // Angle calculation from tools/laser.ipynb
        final double pivotAngle = 2.435184375290124;
        final double d = 0.1711585522257068;

        // Length from Astrobee pivot to target
        double l = Math.sqrt(Math.pow(l2t, 2) + Math.pow(yc, 2));

        double lp = Math.sqrt(Math.pow(l2t-navX, 2) + Math.pow(yc-navY, 2));

        double angle1  = Math.acos((Math.pow(d, 2) + Math.pow(l, 2) - Math.pow(lp, 2))/(2*d*l));
        double angle2  = Math.toRadians(180) - pivotAngle - Math.asin((d*Math.sin(pivotAngle))/l);

        String TAG = "computeXAngle";
        Log.i(TAG, "l = " + l);
        Log.i(TAG, "lp = " + lp);
        Log.i(TAG, "angle1 = " + angle1);
        Log.i(TAG, "angle2 = " + angle2);
        Log.i(TAG, "turn_angle = " + (angle1 - angle2));

        return angle1 - angle2;
    }

    // left angle
    private double computeYAngle(double xc, double l2t) {
        final double navX = 0.0572;
        final double navY = 0.1302;

        // Angle calculation from tools/laser.ipynb
        final double pivotAngle = 2.727652176143348;
        final double d = 0.14221068876846074;

        // Length from Astrobee pivot to target
        double l = Math.sqrt(Math.pow(xc, 2) + Math.pow(l2t, 2));

        double lp = Math.sqrt(Math.pow(xc - navX, 2) + Math.pow(l2t - navY, 2));

        double angle1  = Math.acos((Math.pow(d, 2) + Math.pow(l, 2) - Math.pow(lp, 2))/(2*d*l));
        double angle2  = Math.toRadians(180) - pivotAngle - Math.asin((d*Math.sin(pivotAngle))/l);

        String TAG = "computeYAngle";
        Log.i(TAG, "l = " + l);
        Log.i(TAG, "lp = " + lp);
        Log.i(TAG, "angle1 = " + angle1);
        Log.i(TAG, "angle2 = " + angle2);
        Log.i(TAG, "turn_angle = " + (angle1 - angle2));

        return angle1 - angle2;
    }

    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
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

        return new Quaternion((float) x, (float) y, (float) z, (float) w);
    }

    private Quaternion mutQuaternion(Quaternion q1, Quaternion q2) {
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

    @Override
    protected void runPlan1() {
        api.startMission();

        // move to point A
        Quaternion quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
        move_to(10.71, -7.7, 4.48, quaternion); // test quaternion
        api.reportPoint1Arrival();

        // shoot laser
        move_to(10.71, -7.811971, 4.48, quaternion);
        api.laserControl(true);
        api.takeTarget1Snapshot();


        // move to point 2
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        move_to(10.876214285714285, -8.5, 4.97063, quaternion);
        move_to(11.0067, -9.44819, 5.186407142857143, quaternion);

        Point B =  new Point(11.2746, -9.92284, 5.29881);
        move_to(B, quaternion);

        // shoot laser
        try {
            Thread.sleep(6000);
        } catch (Exception e) {}

        // finding target
        final int cX = 500;
        final int cY = 500;
        Mat img = new Mat(api.getMatNavCam(), new Rect(cX, cY, 330, 200));
        Mat ud_img = undistortCroppedImg(img, cX, cY);
        Target target = new Target(ud_img, cX, cY);
        api.saveMatImage(ud_img, "ud_img");

        // compute angle
        final double l2t = Math.abs(-10.581 - (B.getY()));
        final double xAngle = -computeXAngle(l2t, target.getY());
        final double yAngle = -computeYAngle(target.getX(), l2t);

        Quaternion relativeQ = eulerAngleToQuaternion(xAngle, 0, yAngle);
        Quaternion absoluteQ = mutQuaternion(relativeQ, quaternion);

        String TAG = "Rotation";
        Log.i(TAG, "relativeQ = " + relativeQ.toString());
        Log.i(TAG, "absoluteQ = " + absoluteQ.toString());

        // try rotating to wanted angle and shoot laser
        int retry = 0;
        while (retry <= LOOP_MAX) {
            move_to(B, absoluteQ); // rotate astrobee

            // compare result with what we need
            Kinematics res = api.getRobotKinematics();
            double angleDiff = compareQuaternion(absoluteQ, res.getOrientation());

            Log.i(TAG, String.format("angleDiff = %f", angleDiff));
            Log.i(TAG, String.format("Result %d = %s, %s, %s",
                    retry,
                    res.getConfidence(),
                    res.getPosition().toString(),
                    res.getOrientation().toString()
            ));

            if (angleDiff < 1) {
                try {
                    Thread.sleep(500);
                } catch (Exception e) {}
                api.laserControl(true);
                break;
            }
            else if (retry == 3) {
                // last resort shooting
                try {
                    Thread.sleep(500);
                } catch (Exception e) {}
                api.laserControl(true);
                break;
            }

            move_to(B, quaternion);; // reset astrobee
            retry++;
        }

        api.takeTarget2Snapshot();

        // move to goal
        move_to(11.0067, -9.44819, 5.1722, quaternion);
        move_to(11.073773469387755, -8.5, 4.97063, quaternion);
        move_to(11.2746, -7.89178, 4.96538, quaternion);

        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }
}

