package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Core;

import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

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

            Mat img = new Mat(api.getMatNavCam(), new Rect(cX, cY, 330, 200));
            Mat ud_img = undistortCroppedImg(img, cX, cY);
            double meterPx = computeMeterPx(ud_img);
            double[] target = findCircle(ud_img, cX, cY);

            double[] center = {635.434258, 500.335102}; // principal point
            this.xc = ((target[0] - center[0]) * meterPx) - 0.0422;
            this.yc = -((target[1] - center[1]) * meterPx) + 0.0826;
            this.tL = Math.abs(-10.581 - (-9.922840));


            api.saveMatImage(api.getMatNavCam(), "target2");
            api.saveMatImage(img, "target2_c");
            api.saveMatImage(ud_img, "target2_ud");
            String TAG = "Target";
            Log.i(TAG, "meterPerPixel = " + meterPx);
            Log.i(TAG, "target = " + target);
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
    public double computeXAngle(double tL, double yc) {
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
    public double computeYAngle(double xc, double tL) {
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

        String TAG = "computeXAngle";
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


    private Quaternion combineQuaternion(Quaternion newOrientation, Quaternion oldOrientation) {
        // For multiply quaternion, Apply q2(new) to q1(old)
        // q1 * q2= a*e - b*f - c*g- d*h + i (b*e + a*f + c*h - d*g) + j (a*g - b*h + c*e + d*f) + k (a*h + b*g - c*f + d*e)

        double x = newOrientation.getX() * oldOrientation.getW() + newOrientation.getY() * oldOrientation.getZ()
                - newOrientation.getZ() * oldOrientation.getY() + newOrientation.getW() * oldOrientation.getX();
        double y = -newOrientation.getX() * oldOrientation.getZ() + newOrientation.getY() * oldOrientation.getW()
                + newOrientation.getZ() * oldOrientation.getX() + newOrientation.getW() * oldOrientation.getY();
        double z = newOrientation.getX() * oldOrientation.getY() - newOrientation.getY() * oldOrientation.getX()
                + newOrientation.getZ() * oldOrientation.getW() + newOrientation.getW() * oldOrientation.getZ();
        double w = -newOrientation.getX() * oldOrientation.getX() - newOrientation.getY() * oldOrientation.getY()
                - newOrientation.getZ() * oldOrientation.getZ() + newOrientation.getW() * oldOrientation.getW();

        return new Quaternion((float) x, (float) y, (float) z, (float) w);
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
            Thread.sleep(5000);
        } catch (Exception e) {}

        Target target = new Target();
        double[] c = target.getCordinates();
        final double xAngle = -computeXAngle(c[2], c[1]);
        final double yAngle = -computeYAngle(c[0], c[2]);

        Quaternion imageQ = eulerAngleToQuaternion(xAngle, 0, yAngle - 0.03); // 0.03 magical offset
        Quaternion qToTurn = combineQuaternion(imageQ, quaternion);

        move_to(11.2746, -9.92284, 5.29881, qToTurn);
        api.laserControl(true);

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

