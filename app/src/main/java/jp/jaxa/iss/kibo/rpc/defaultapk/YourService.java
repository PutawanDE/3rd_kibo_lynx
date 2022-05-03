package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Board;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
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

    final double[] CAM_POS = {
            0.1177, -0.0422, -0.0826
    };

    final double[] LASER_POS = {
            0.1302, 0.0572, -0.1111
    };

    final double target2_marker_x_dist = 0.225;
    final double target2_marker_y_dist = 0.083;

    final double[] marker12_point = {0, 0, 0};
    final double[] marker11_point = {target2_marker_x_dist, 0, 0};
    final double[] marker14_point = {target2_marker_x_dist, target2_marker_y_dist, 0};
    final double[] marker13_point = {0, target2_marker_y_dist, 0};

    final double[][] markersPoint = {
            marker12_point, marker11_point, marker14_point, marker13_point
    };

    final float MARKER_LENGTH = 0.05f;

    private void move_to(double x, double y, double z, Quaternion q) {
        final Point p = new Point(x, y, z);

        int counter = 0;
        Result result;

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

    }

    private void relative_move_to(double x, double y, double z, Quaternion q) {
        final Point p = new Point(x, y, z);
        int counter = 0;
        Result result;

        do {
            result = api.relativeMoveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));
    }

    private double[] estimateBoardPos(Mat img) {
        final Dictionary ARUCO_DICT = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        final String TAG = "estimateBoardPos";
        api.saveMatImage(img, "ReadAr.png");
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        List<Mat> corners = new ArrayList<>();
        Mat detectedIDs = new Mat();
        Mat tvec = new Mat();
        Mat rvec = new Mat();
        double[] boardPos = new double[3];

        try {
            Log.i(TAG, "Reading AR tags");
            int counter = 0;
            do {
                Aruco.detectMarkers(img, ARUCO_DICT, corners, detectedIDs);
                counter++;
                Log.i(TAG, "Reading AR tags Attempt#" + counter);
            } while (corners.size() != 4 && counter < LOOP_MAX);

            String status = corners.size() == 4 ? "Success" : "Fail";
            Log.i(TAG, "Reading AR tags " + status);
            Log.i(TAG, "AR ids: " + detectedIDs.dump());
            for (Mat c : corners) {
                Log.i(TAG, "AR pos: " + c.dump());
            }

            Mat detectedMarkerImg = new Mat();
            Imgproc.cvtColor(img, detectedMarkerImg, Imgproc.COLOR_GRAY2RGB);
            Aruco.drawDetectedMarkers(detectedMarkerImg, corners);
            api.saveMatImage(detectedMarkerImg, "ArucoDetectedMarker.png");

            Mat boardIDs = new Mat(4, 1, CvType.CV_8U);
            boardIDs.put(0, 0, 12);
            boardIDs.put(1, 0, 11);
            boardIDs.put(2, 0, 14);
            boardIDs.put(3, 0, 13);

            Board board = Board.create(initializeTarget2AR_points(), ARUCO_DICT, boardIDs);
            for (MatOfPoint3f o : board.get_objPoints()) {
                Log.i(TAG, "Marker object points: " + o.dump());
            }

            Aruco.estimatePoseBoard(corners, detectedIDs, board, cameraMat, distCoeffs, rvec, tvec);
            Log.i(TAG, "Board tvec: " + tvec.dump());
            Log.i(TAG, "Board rvec: " + rvec.dump());

            Aruco.drawAxis(detectedMarkerImg, cameraMat, distCoeffs, rvec, tvec, MARKER_LENGTH);
            api.saveMatImage(detectedMarkerImg, "ArucoBoardPose.png");

            double tvec_x = tvec.get(0, 0)[0];
            double tvec_y = tvec.get(1, 0)[0];
            double tvec_z = tvec.get(2, 0)[0];

            boardPos = new double[]{
                    tvec_z + CAM_POS[0],
                    tvec_x + CAM_POS[1],
                    tvec_y + CAM_POS[2],
            };

            Log.i(TAG, "Board position relative to Astrobee: {" +
                    boardPos[0] + ", " + boardPos[1] + ", " + boardPos[2] + "}");
        } catch (Exception e) {
            e.printStackTrace();
            Log.i(TAG, "--Fail: " + e.toString());
        }

        return boardPos;
    }

    private List<Mat> initializeTarget2AR_points() {
        List<Mat> markerObjPoints = new ArrayList<>();
        int markersCnt = 4;
        for (int i = 0; i < markersCnt; i++) {
            Point3 topLeft = new Point3(markersPoint[i]);
            Point3 topRight = new Point3(markersPoint[i][0] + MARKER_LENGTH,
                    markersPoint[i][1], markersPoint[i][2]);
            Point3 bottomRight = new Point3(markersPoint[i][0] + MARKER_LENGTH,
                    markersPoint[i][1] + MARKER_LENGTH, markersPoint[i][2]);
            Point3 bottomLeft = new Point3(markersPoint[i][0],
                    markersPoint[i][1] + MARKER_LENGTH, markersPoint[i][2]);

            MatOfPoint3f singleMarkerObjPoint = new MatOfPoint3f(topLeft, topRight, bottomRight,
                    bottomLeft);
            markerObjPoints.add(singleMarkerObjPoint);
        }
        return markerObjPoints;
    }

    private Mat find_circle() {
        Mat circle = new Mat();
        Mat img = new Mat(api.getMatNavCam(), new Rect(550, 500, 300, 300));

        Imgproc.HoughCircles(img, circle, Imgproc.HOUGH_GRADIENT, 1, 300, 50, 30, 0, 0);

        org.opencv.core.Point center = new org.opencv.core.Point(circle.get(0, 0)[0], circle.get(0, 0)[1]);

        if (circle.empty())
            return null;

        Mat res = new Mat(1, 2, CvType.CV_32FC2);
        float[] point = {(float) center.x + 550, (float) center.y + 500};

        res.put(0, 0, point);

        return res;
    }

    private Mat undistortPoints(Mat points) {
        // in -> rows:1, cols:4
        // in -> 1xN 2 Channel

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        Mat out = new Mat(points.rows(), points.cols(), points.type());

        Imgproc.undistortPoints(points, out, cameraMat, distCoeffs, new Mat(), cameraMat);

        // out -> 1xN 2 Channel
        return out;
    }

    private double[] pixelDistanceToAngle(double[] target, double[] ref) {

        final double xDistance = target[0] - ref[0];
        final double yDistance = ref[1] - target[1];
        //final double anglePerPixel = 130 / Math.sqrt(Math.pow(NAV_MAX_WIDTH, 2) + Math.pow(NAV_MAX_HEIGHT, 2));
        final double anglePerPixel = 0.08125;

        double xAngle = xDistance * anglePerPixel;
        double yAngle = yDistance * anglePerPixel;

        double[] out = {xAngle, yAngle};
        return out;
    }

    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
        xAngle = Math.toRadians(xAngle);
        yAngle = Math.toRadians(yAngle);
        zAngle = Math.toRadians(zAngle);
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
        move_to(10.71, -7.7, 4.48, quaternion);
        api.reportPoint1Arrival();

        // shoot laser
        move_to(10.71, -7.811971, 4.48, quaternion);
        api.laserControl(true);
        api.takeTarget1Snapshot();


        // move to point 2
        quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        move_to(10.876214285714285, -8.5, 4.97063, quaternion);
        move_to(11.0067, -9.44819, 5.186407142857143, quaternion);
        move_to(11.2746, -10.0, 5.29881, quaternion);

        // shoot laser
        final Mat target = find_circle();
//        final Mat undistort_target = undistortPoints(target);
//        final double[] laser = {711, 455};
//        final double[] angleToTurn = pixelDistanceToAngle(undistort_target.get(0, 0), laser);
//        final Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
//        final Quaternion qToTurn  = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
//
//        move_to(11.2746, -9.92284, 5.29881, qToTurn);

        double[] arBoardPos = estimateBoardPos(api.getMatNavCam());

        relative_move_to(arBoardPos[1] - LASER_POS[1], 0, arBoardPos[2] - LASER_POS[2],
                quaternion);

        api.laserControl(true);
        api.saveMatImage(api.getMatNavCam(), "ShootLaser.png");
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

