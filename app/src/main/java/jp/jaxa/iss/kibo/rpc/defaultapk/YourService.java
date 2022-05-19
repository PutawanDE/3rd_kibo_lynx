package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

    private void move_to(double x, double y, double z, Quaternion q) {
        final Point p = new Point(x, y, z);

        int counter = 0;
        Result result;

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));
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

    private double distance(double x0, double y0, double x1, double y1) {
        return Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));
    }

    class Target {
        private double xc;
        private double yc;
        private double tL;

        public Target() {
            final int cX = 500;
            final int cY = 500;
            final double xOffset = 0.019;
            final double yOffset = 0.016;

            Mat img = new Mat(api.getMatNavCam(), new Rect(cX, cY, 330, 200));
            Mat ud_img = undistortCroppedImg(img, cX, cY);
            double meterPx = computeMeterPx(ud_img);
            double[] target = findCircle(ud_img, cX, cY);

            double[] center = {635.434258, 500.335102}; // principal point
            this.xc = ((target[0] - center[0]) * meterPx) - 0.0422 - xOffset;
            this.yc = -((target[1] - center[1]) * meterPx) + 0.0826 - yOffset;
            this.tL = Math.abs(-10.581 - (-9.922840));


            api.saveMatImage(api.getMatNavCam(), "target2");
            api.saveMatImage(img, "target2_c");
            api.saveMatImage(ud_img, "target2_ud");
            String TAG = "Target";
            Log.i(TAG, "meterPerPixel = " + meterPx);
            Log.i(TAG, "target = " + Arrays.toString(target));
            Log.i(TAG, "xc, yc, tL = " + xc + ", " + yc + ", " + tL);
        }

        public double[] getCordinates() {
            return new double[]{xc, yc, tL};
        }

        private double computeMeterPx(Mat ud_img) {
            String TAG = "computeMeterPx";

            Mat ids = new Mat();
            List<Mat> corners = new ArrayList<>();
            final Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

            int counter = 0;
            while (ids.rows() < 4 && counter < LOOP_MAX) { // try 3 until find all 4 ids
                Aruco.detectMarkers(ud_img, dict, corners, ids); // find all ids
                counter++;
            }

            for (int i = 0; i < corners.size(); i++) {
                Mat p = corners.get(i);
                for (int j = 0; j < p.cols(); j++) {
                    double[] t = p.get(0, j);
                    t[0] += 350;
                    t[1] += 350;

                    p.put(0, j, t[0], t[1]);
                }
            }

            double sLen = 0;
            for (int i = 0; i < corners.size(); i++) {
                Mat p = corners.get(i);
                double s1 = distance(p.get(0, 0)[0], p.get(0, 0)[1], p.get(0, 1)[0], p.get(0, 1)[1]);
                double s2 = distance(p.get(0, 1)[0], p.get(0, 1)[1], p.get(0, 2)[0], p.get(0, 2)[1]);
                double s3 = distance(p.get(0, 2)[0], p.get(0, 2)[1], p.get(0, 3)[0], p.get(0, 3)[1]);
                double s4 = distance(p.get(0, 3)[0], p.get(0, 3)[1], p.get(0, 0)[0], p.get(0, 0)[1]);
                double sAvg = (s1 + s2 + s3 + s4)/4;
                sLen += sAvg;
            }

            Log.i(TAG, "sLen  = " + sLen / corners.size());

            return 0.05 / (sLen / corners.size());
        }

        private double[] findCircle(Mat ud_img, int x, int y) {
            Mat circle = new Mat();
            Imgproc.HoughCircles(ud_img, circle, Imgproc.HOUGH_GRADIENT, 1, 300, 50, 30, 40, 50);

            if (circle.empty())
                return null;

            double[] point = {circle.get(0, 0)[0] + x, circle.get(0, 0)[1] + y};
            return point;
        }
    }

    // down up
    private double computeXAngle(double tL, double yc) {
        double navX = 0.1302;
        double navY = 0.1111;

        // Angle calculation from tools/laser.ipynb
        double pivotAngle = 2.435184375290124;

        double d = 0.1711585522257068;

        // Length from Astrobee pivot to target
        double l = Math.sqrt(Math.pow(tL, 2) + Math.pow(yc, 2));

        double lp = Math.sqrt(Math.pow(tL-navX, 2) + Math.pow(yc-navY, 2));

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

    // left right
    private double computeYAngle(double xc, double tL) {
        double navX = 0.0572;
        double navY = 0.1302;

        // Angle calculation from tools/laser.ipynb
        double pivotAngle = 2.727652176143348;

        double d = 0.14221068876846074;

        // Length from Astrobee pivot to target
        double l = Math.sqrt(Math.pow(xc, 2) + Math.pow(tL, 2));

        double lp = Math.sqrt(Math.pow(xc - navX, 2) + Math.pow(tL - navY, 2));

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

    private double vectorDot(double[] v1, double[] v2) {
        double sum = 0;
        for (int i = 0; i < v1.length; i++) {
            sum += v1[i] * v2[i];
        }
        return sum;
    }

    private double[] scalarDot(double s, double[] v2) {
        double[] res = new double[v2.length];
        for (int i = 0; i < v2.length; i++) {
            res[i] = s * v2[i];
        }
        return res;
    }

    private double[] vectorSum(double[] v1, double[] v2) {
        // Vector must be same size
        double[] res = new double[v2.length];
        for (int i = 0; i < v1.length; i++) {
            res[i] = v1[i] + v2[i];
        }
        return res;
    }

    private double[] vectorCross(double[] v1, double[] v2) {
        // Vector must have length 3
        double[] res = new double[3];
        res[0] = v1[1] * v2[2] - v1[2] * v2[1];
        res[1] = v1[2] * v2[0] - v1[0] * v2[2];
        res[2] = v1[0] * v2[1] - v1[1] * v2[0];
        return res;
    }

    private double vectorSize(double[] v) {
        double sum = 0;
        for (int i = 0; i < v.length; i++) {
            sum += Math.pow(v[i], 2);
        }
        return Math.sqrt(sum);
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
        double[] v1 = new double[]{q1.getX(), q1.getY(), q1.getZ()};
        double s2 = q2.getW();
        double[] v2 = new double[]{q2.getX(), q2.getY(), q2.getZ()};

        double[] a = vectorSum(scalarDot(s1, v2), scalarDot(s2, v1));
        double[] b = vectorCross(v1, v2);
        double[] res = vectorSum(a, b);
        double w = s1*s2 - vectorDot(v1, v2);
        return new Quaternion((float) res[0], (float) res[1], (float) res[2], (float) w);
    }

    private double[] vectorOrientation(Quaternion q) {
        double[] v = new double[]{1, 0, 0}; // (0, 0, 0, 1) quaternion -> vector
        double[] u = new double[]{q.getX(), q.getY(), q.getZ()};
        double s = q.getW();

        double[] a = scalarDot(2.0 * vectorDot(u, v), u);
        double[] b = scalarDot(s * s - vectorDot(u, u), v);
        double[] c = scalarDot(2.0 * s, vectorCross(u, v));
        return vectorSum(vectorSum(a, b), c);
    }

    private double vectorAngleDiff(double[] v1, double[] v2) {
        // Vector must have length 3
        double a = vectorDot(v1, v2);
        double b = vectorSize(v1) * vectorSize(v2);
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
        move_to(10.71, -7.7, 4.48, new Quaternion(0, 0, 0, 1)); // test quaternion
        api.reportPoint1Arrival();

        // shoot laser
        move_to(10.71, -7.811971, 4.48, quaternion);
        api.laserControl(true);
        api.takeTarget1Snapshot();


        // move to point 2
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        move_to(10.876214285714285, -8.5, 4.97063, quaternion);
        move_to(11.0067, -9.44819, 5.186407142857143, quaternion);
        move_to(11.2746, -9.92284, 5.29881, quaternion);

        // shoot laser
        try {
            Thread.sleep(6000);
        } catch (Exception e) {}

        String TAG = "Rotation";
        Target target = new Target();
        double[] c = target.getCordinates();
        final double xAngle = -computeXAngle(c[2], c[1]);
        final double yAngle = -computeYAngle(c[0], c[2]);

        Quaternion relativeQ = eulerAngleToQuaternion(xAngle, 0, yAngle);
        Quaternion absoluteQ = mutQuaternion(relativeQ, quaternion);

        Log.i(TAG, "relativeQ = " + relativeQ.toString());
        Log.i(TAG, "absoluteQ = " + absoluteQ.toString());

        int retry = 0;
        boolean alrShoot = false;
        while (retry < LOOP_MAX) {
            move_to(11.2746, -9.92284, 5.29881, absoluteQ); // rotate astrobee

            // compare result with what we need
            Kinematics res = api.getRobotKinematics();
            double angleDiff = compareQuaternion(absoluteQ, res.getOrientation());

            Log.i(TAG, String.format("angleDiff = %f", angleDiff));

            if (angleDiff < 1.5) {
                try {
                    Thread.sleep(500);
                } catch (Exception e) {}
                alrShoot = true;
                api.laserControl(true);
                break;
            }

            move_to(11.2746, -9.92284, 5.29881, quaternion); // reset astrobee

            retry++;
            Log.i(TAG, String.format("Result %d = %s, %s, %s",
                    retry,
                    res.getConfidence(),
                    res.getPosition().toString(),
                    res.getOrientation().toString()
            ));
        }

        if (!alrShoot) {
            // Last resort shooting
            move_to(11.2746, -9.92284, 5.29881, absoluteQ); // rotate astrobee
            api.laserControl(true);
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

