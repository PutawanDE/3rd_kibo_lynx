package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Core;

import org.opencv.imgproc.Imgproc;

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

    final double[] NEWCAM_MATSIM = {
            347.69958496, 0., 633.39499987,
            0., 357.13922119, 494.69674079,
            0., 0., 1.
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

    private Mat find_circle() {
        Mat circle = new Mat();
        Mat img = new Mat(api.getMatNavCam(), new Rect(550, 500, 300, 300));
        api.saveMatImage(api.getMatNavCam(), "target2");
        api.saveMatImage(img, "target2_c");

        Imgproc.HoughCircles(img, circle, Imgproc.HOUGH_GRADIENT, 1, 300, 50, 30, 0, 0);

        org.opencv.core.Point center = new org.opencv.core.Point(circle.get(0, 0)[0], circle.get(0, 0)[1]);

        if (circle.empty())
            return null;

        Mat res = new Mat(1, 2, CvType.CV_32FC2);
        float[] point = {(float) center.x + 550, (float) center.y + 500};


        res.put(0, 0, point);

        return res;
    }

    /**
     * @param points 1xN 2 Channel
     * @return 1xN 2 Channel
     */
    private Mat undistortPoints(Mat points) {
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        Mat newCamMat = new Mat(3, 3, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);
        newCamMat.put(0, 0, NEWCAM_MATSIM);

        Mat out = new Mat(points.rows(), points.cols(), points.type());

        Imgproc.undistortPoints(points, out, cameraMat, distCoeffs, new Mat(), newCamMat);

        return out;
    }

    class Circle {
        private double x, y, radius;

        public Circle(double x, double y, double radius) {
            this.x = x;
            this.y = y;
            this.radius = radius;
        }

        public double[] intersect(Circle c) {
            double d = Math.sqrt(Math.pow(c.x - x, 2) + Math.pow(c.y - y, 2));

            if (d > radius + c.radius) return null;
            if (d < Math.abs(radius - c.radius)) return null;

            if (d == 0 && radius == c.radius) return null;
            else {
                double a = (Math.pow(radius, 2) - Math.pow(c.radius, 2) + Math.pow(d, 2)) / (2 * d);
                double h = Math.sqrt(Math.pow(radius, 2) - Math.pow(a, 2));

                double x2 = x + a * (c.x - x) / d;
                double y2 = y + a * (c.y - y) / d;

                double x3 = x2 + h * (c.y - y) / d;
                double y3 = y2 - h * (c.x - x) / d;

                double x4 = x2 - h * (c.y - y) / d;
                double y4 = y2 + h * (c.x - x) / d;

                return new double[]{x3, y3, x4, y4};
            }
        }
    }

    class AngleCalc {
        private Mat cam_c;

        public AngleCalc(double[] target) {
            double s = 0.45946;
            double[] uvA = {target[0] * s, target[1] * s, 1 * s};
            double[] CamMatInvA = {
                    0.00287605, 0., -1.82167315,
                    0., 0.00280003, -1.38516498,
                    0., 0., 1.
            };

            Mat uv = new Mat(3, 1, CvType.CV_64FC1);
            uv.put(0, 0, uvA);

            Mat CamMatInv = new Mat(3, 3, CvType.CV_64FC1);
            CamMatInv.put(0, 0, CamMatInvA);

            this.cam_c = new Mat(3, 1, CvType.CV_64FC1);
            Core.gemm(CamMatInv, uv, 1, new Mat(3, 3, CvType.CV_64FC1), 0, this.cam_c);

            // Logging
            String TAG = "AngleCalc";
            Log.i(TAG, "target = " + target[0] + ", " + target[1]);
            for (int i = 0; i < 3; i++) {
                Log.i(TAG, "cam_c" + i + " = " + cam_c.get(i, 0)[0]);
            }
        }

        private double dist(double x0, double y0, double x1, double y1) {
            return Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));
        }

        public double computeXAngle() {

            // Angle calculation from tools/laser.ipynb
            double pivotAngle = 2.4351843752901243;
            // Precomputed radius of Astrobee circle
            double r1 = 0.1711585522257068;
            // Length from Astrobee pivot to target
            double l = Math.sqrt(Math.pow(0.5771599999999992, 2) + Math.pow(-(cam_c.get(1, 0)[0] - 0.0826), 2));
            // Radius of target circle
            double r2 = r1 * Math.cos(pivotAngle) + Math.sqrt(Math.pow(l, 2) - Math.pow(r1, 2) * Math.pow(Math.sin(pivotAngle), 2));


            // Find intersection points
            Circle c = new Circle(0, 0, r1);
            double[] intersectP = c.intersect(new Circle(0.5771599999999992, -(cam_c.get(1, 0)[0] - 0.0826), r2));

            // Select the bigger y
            double x = dist(0.1302, 0.1111, intersectP[2], intersectP[3]);

            double angle = 2 * Math.atan((x / 2) / r1);

            // Logging
            String TAG = "computeXAngle";
            Log.i(TAG, "l = " + l);
            Log.i(TAG, "r2 = " + r2);
            for (int i = 0; i < intersectP.length; i++) {
                Log.i(TAG, "intersectP " + i + " = " + intersectP[i]);
            }
            Log.i(TAG, "x = " + x);
            Log.i(TAG, "Angle = " + angle);

            return angle;
        }

        public double computeYAngle() {

            // Angle calculation from tools/laser.ipynb
            double pivotAngle = 1.984736804241342;
            // Precomputed radius of Astrobee circle
            double r1 = 0.14221068876846074;
            // Length from Astrobee pivot to target
            double l = Math.sqrt(Math.pow(0.5771599999999992, 2) + Math.pow(cam_c.get(0, 0)[0] - 0.0422, 2));
            // Radius of target circle
            double r2 = r1 * Math.cos(pivotAngle) + Math.sqrt(Math.pow(l, 2) - Math.pow(r1, 2) * Math.pow(Math.sin(pivotAngle), 2));


            // Find intersection points
            Circle c = new Circle(0, 0, r1);
            double[] intersectP = c.intersect(new Circle(cam_c.get(0, 0)[0] - 0.0422, 0.5771599999999992, r2));

            // Select the bigger x
            double x = dist(0.1302, 0.0572, intersectP[0], intersectP[1]);

            double angle = 2 * Math.atan((x / 2) / r1);

            // Logging
            String TAG = "computeYAngle";
            Log.i(TAG, "l = " + l);
            Log.i(TAG, "r2 = " + r2);
            for (int i = 0; i < intersectP.length; i++) {
                Log.i(TAG, "intersectP " + i + " = " + intersectP[i]);
            }
            Log.i(TAG, "x = " + x);
            Log.i(TAG, "Angle = " + angle);

            return angle;
        }

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
        final Mat target = find_circle();
        final Mat ud_target = undistortPoints(target);

        final AngleCalc ac = new AngleCalc(ud_target.get(0, 0));
        final double xAngle = -ac.computeXAngle();
        final double yAngle = -ac.computeYAngle();

        final Quaternion imageQ = eulerAngleToQuaternion(xAngle, 0, yAngle);
        final Quaternion qToTurn = combineQuaternion(imageQ, quaternion);

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

