package jp.jaxa.iss.kibo.rpc.defaultapk;

import static org.opencv.imgproc.Imgproc.undistort;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Board;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
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

    final double laser_width = 0.0572;
    final double laser_depth = 0.1302;
    final double laser_oblique_x = 0.14221;
    final double laser_oblique_y = 0.171159;
    final double laser_high = 0.1111;

    final double cam_depth = 0.1177;
    final double cam_width = 0.0422;
    final double cam_high = 0.0826;

    final double target2_marker_x_dist = 0.225;
    final double target2_marker_y_dist = 0.083;
    final double target2_board_width = 0.275;
    final double target2_board_height = 0.133;

    final double[] marker12_point = {0, 0, 0};
    final double[] marker11_point = {target2_marker_x_dist, 0, 0};
    final double[] marker14_point = {target2_marker_x_dist, target2_marker_y_dist, 0};
    final double[] marker13_point = {0, target2_marker_y_dist, 0};

    final double[][] markersPoint = {
            marker12_point, marker11_point, marker14_point, marker13_point
    };

    final float MARKER_LENGTH = 0.05f;

    long debug_Timestart = 0;
    private boolean FAIL_to_Find_TARGET;

    @Override
    protected void runPlan1() {
        debug_Timestart = System.currentTimeMillis();
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
        move_to(11.2746, -9.92284, 5.29881, quaternion);

        // shoot laser
//        final Mat target = find_circle();
//        final Mat undistort_target = undistortPoints(target);
//        final double[] laser = {711, 455};
//        final double[] angleToTurn = pixelDistanceToAngle(undistort_target.get(0, 0), laser);
//        final Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
//        final Quaternion qToTurn  = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
//
//        move_to(11.2746, -9.92284, 5.29881, qToTurn);

        // Wait for camera delay or Astrobee to stop moving due to inertia
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double cam2WallDis = estimateCam2WallDis(api.getMatNavCam());
        Mat cropPic = new Mat(undistortPic(api.getMatNavCam()), new Rect(350, 350, 600, 360));
        ArBoard undistortedBoard = readAR(cropPic);
        double meterPerPixel = calcMeterPerPxFromBoard(undistortedBoard);
        double[] target = findTarget(cropPic);

        double[] angleToTurn = pixelDistanceToAngle(target, cam2WallDis, meterPerPixel);
        Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
        Quaternion qToTurn_Target2 = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
        turn_to(qToTurn_Target2);

//        relative_move_to(arBoardPos[1] - LASER_POS[1], 0, arBoardPos[2] - LASER_POS[2],
//                quaternion);
//        double[] arBoardPos = estimateBoardPosAve(api.getMatNavCam(), 5);

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

    private void move_to(double x, double y, double z, Quaternion q) {
        final Point p = new Point(x, y, z);

        int counter = 0;
        Result result;

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

    }

    private void turn_to(Quaternion q) {
        final String TAG = "turn_to";
        final Point p_dormant = new Point(0, 0, 0);

        int counter = 0;
        Result result;

        Log.i(TAG, "Start turn to p:" + p_dormant + ", q:" + q);

        do {
            result = api.relativeMoveTo(p_dormant, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        Log.i(TAG, "Done turn to p:" + p_dormant + ", q:" + q);

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

    private double calcMeterPerPxFromBoard(ArBoard undistortedBoard) {
        final String TAG = "calcPixelPerMeterFromBoard";
        int[] dimensions = undistortedBoard.getDimensionPx();
        if (dimensions.length == 4) {
            int top = dimensions[0];
            int right = dimensions[1];
            int bottom = dimensions[2];
            int left = dimensions[3];
            double realWidth = undistortedBoard.getRealWidth();
            double realHeight = undistortedBoard.getRealHeight();

            Log.i(TAG, "Board Dimension in Px: " + Arrays.toString(dimensions));
            Log.i(TAG, "Board Dimension in m. (Real): w=" + realWidth + " h=" + realHeight);

            double meterPerPx = ((realWidth / top) + (realWidth / bottom) +
                    (realHeight / right) + (realHeight / left)) / 4.0;
            Log.i(TAG, "Pixel per meter= " + meterPerPx);
            return meterPerPx;
        } else {
            //TODO find the best fixed pixel per meter
            double fixedPixelPerMeter = 0;
            Log.i(TAG, "Detected Board is incomplete. Returning Fixed PixelPerMeter= " +
                    fixedPixelPerMeter);
            return fixedPixelPerMeter;
        }
    }

    private double estimateCam2WallDis(Mat img) {
        final Dictionary ARUCO_DICT = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        final String TAG = "estimateBoardPos";
        api.saveMatImage(img, "Input_Board_estimate.png");

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        try {
            Log.i(TAG, "Read AR for ARUCO board pose estimation");
            ArBoard detectedBoard = readAR(img);
            List<Mat> corners = detectedBoard.getCorners();
            Mat detectedIDs = detectedBoard.getIds();
            Log.i(TAG, "Reading AR DONE########");

            Mat tvec = new Mat();
            Mat rvec = new Mat();

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
            double tvec_z = tvec.get(2, 0)[0];
            return tvec_z;
        } catch (Exception e) {
            Log.e(TAG, "Fail to estimate board pose", e);
            // TODO find the best fixedCam2WallDis
            double fixedCam2WallDis = 0.47;
            Log.i(TAG, "Due to failure, returning fixed Cam2WallDis= " + fixedCam2WallDis);
            return fixedCam2WallDis;
        }
    }

    private ArBoard readAR(Mat img) {
        final Dictionary ARUCO_DICT = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        final String TAG = "getMarkerCorners";
        api.saveMatImage(img, "Input_readAR.png");

        List<Mat> corners = new ArrayList<>();
        Mat detectedIDs = new Mat();

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
        } catch (Exception e) {
            Log.e(TAG, "Fail reading AR tags", e);
        }

        return new ArBoard(corners, detectedIDs, target2_board_width, target2_board_height);
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

    private double[] findTarget(Mat pic) {
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

            Log.i(TAG, "Succeed findTargetCenter =" + Arrays.toString(point));
            return point;
        } catch (Exception e) {
            //######################
            FAIL_to_Find_TARGET = true;
            //######################
            Log.e(TAG, "findTargetCenter_Cycle", e);
            // TODO find the best fixed point
            double[] point = {-1, -1}; // fixed point
            Log.i(TAG, "Can't find target. Returning fixed target point= " + Arrays.toString(point));
            return point;
        }
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

    private Mat undistortPic(Mat pic) {
        final String TAG = "undistortPic";
        api.saveMatImage(pic, (System.currentTimeMillis() - debug_Timestart) + " Input Pic to undistort.png");

        Log.i(TAG, "Start");

        try {
            Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
            Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

            cameraMat.put(0, 0, CAM_MATSIM);
            distCoeffs.put(0, 0, DIST_COEFFSIM);
            Mat out = new Mat(pic.rows(), pic.cols(), pic.type());

            undistort(pic, out, cameraMat, distCoeffs);

            api.saveMatImage(out, (System.currentTimeMillis() - debug_Timestart) + " undistort pic.png");
            Log.i(TAG, "Succeed undistort");
            return out;
        } catch (Exception e) {
            Log.i(TAG, "Fail undistort " + e);
            return pic;
        }
    }

    private double[] pixelDistanceToAngle(double[] target, double cam2walldis, double meter_perPx) {
        final String TAG = "pixelDistanceToAngle";
        Log.i(TAG, "================== pixelDistanceToAngle ==================");
        //double[] ref  = {640, 480};
        double[] ref = {635.434258, 500.335102};
        double xDistance = (target[0] + 350) - ref[0];
        double yDistance = ref[1] - (target[1] + 350);

//        double cam2walldis = focusCamera * 5 / arTag_sizePx;
//        final double cam2walldis = 0.56 ; // focusCamera = 4.161542


        Log.i(TAG, "xDistance=" + xDistance);
        Log.i(TAG, "yDistance=" + yDistance);
        Log.i(TAG, "cam2walldis=" + cam2walldis);

        double k2w = cam2walldis + cam_depth;
        double l2w = cam2walldis - (laser_depth - cam_depth);
        Log.i(TAG, "k2w = " + k2w);
        Log.i(TAG, "l2w = " + l2w);


        double cam_target_dis_x = (xDistance * meter_perPx);
        double cam_target_dis_y = (yDistance * meter_perPx);
        Log.i(TAG, "cam_target_dis (x)= " + cam_target_dis_x);
        Log.i(TAG, "cam_target_dis (y)= " + cam_target_dis_y);

        //########  horizonAngle axis #######
        double horizonAngle = horizonAngle_axis(cam_target_dis_x, k2w, l2w);
        Log.i(TAG, "horizonAngle(x)= " + horizonAngle);
        //###################################

        //########  verticalAngle axis #######
        double verticalAngle = verticalAngle_axis(cam_target_dis_y, k2w, l2w);
        Log.i(TAG, "verticalAngle(y)= " + verticalAngle);
        //###################################

        double[] out = {horizonAngle, verticalAngle};
        Log.i(TAG, "Angle(x,y)= " + Arrays.toString(out));
        Log.i(TAG, "================== Done ==================");
        return out;
    }

    private double horizonAngle_axis(double cam_target_dis_x, double k2w, double l2w) {
        final String TAG = "horizonAngle_axis";
        double horizonAngle;
        double A_angel = 0;
        double k2t_x = cam_target_dis_x - cam_width;
        double l2t_x = cam_target_dis_x - (cam_width + laser_width);
        Log.i(TAG, "kibo 2 target _x(x)= " + k2t_x);
        Log.i(TAG, "laser 2 target _x(x)= " + l2t_x);

        double ok2t_dx = Math.sqrt((k2w * k2w) + (k2t_x * k2t_x));
        Log.i(TAG, "ok2t_dx= " + ok2t_dx);
        double ol2t_dx = Math.sqrt((l2w * l2w) + (l2t_x * l2t_x));
        Log.i(TAG, "ol2t_dx= " + ol2t_dx);
        double orl2t_dx = Math.sqrt(Math.abs((ok2t_dx * ok2t_dx) - (laser_width * laser_width))) - laser_depth;
        Log.i(TAG, "orl2t_dx= " + orl2t_dx);

        double cosVal_A_R = ((orl2t_dx * orl2t_dx) + (ok2t_dx * ok2t_dx) - (laser_oblique_x * laser_oblique_x)) / (2 * orl2t_dx * ok2t_dx);
        Log.i(TAG, "cosVal_A_R= " + cosVal_A_R);

        double A_R = Math.toDegrees(Math.acos(cosVal_A_R));
        Log.i(TAG, "A_R (x)= " + A_R);

        double A_K = Math.toDegrees(Math.atan(k2w / Math.abs(k2t_x)));
        Log.i(TAG, "A_K (x)= " + A_K);

        double A_L = Math.toDegrees(Math.atan(l2w / Math.abs(l2t_x)));
        Log.i(TAG, "A_L (x)= " + A_L);

        if (k2t_x < 0) {
            Log.i(TAG, "target on Left  of kibo : " + l2t_x);
            A_angel = A_K - (A_R + A_L);
        } else {
            if (l2t_x < 0) {
                Log.i(TAG, "target on Left  of laser : " + l2t_x);
                A_angel = 180 - (A_R + A_L + A_K);
            } else if (l2t_x > 0) {
                Log.i(TAG, "target on Right of laser : " + l2t_x);
                A_angel = A_R - (A_L - A_K);
            }
        }
        Log.i(TAG, "A_angel= " + A_angel);

        double X_dis = Math.sqrt((ol2t_dx * ol2t_dx) + (orl2t_dx * orl2t_dx) - 2 * ol2t_dx * orl2t_dx * Math.cos(Math.toRadians(A_angel)));
        Log.i(TAG, "X_dis= " + X_dis);
        horizonAngle = (Math.toDegrees(Math.asin((X_dis / 2) / laser_oblique_x))) * 2;

        if (l2t_x < 0) {
            Log.i(TAG, "horizonAngle are negative");
            horizonAngle = horizonAngle * -1;
        }

        Log.i(TAG, "horizonAngle= " + horizonAngle);

        return horizonAngle;
    }

    private double verticalAngle_axis(double cam_target_dis_y, double k2w, double l2w) {
        final String TAG = "verticalAngle_axis";
        double verticalAngle;
        double A_angel = 0;

        double k2t_y = cam_target_dis_y + cam_high;
        double l2t_y = cam_target_dis_y - (laser_high - cam_high);
        Log.i(TAG, "kibo 2 target _y(y)= " + k2t_y);
        Log.i(TAG, "laser 2 target _y(y)= " + l2t_y);


        double ok2t_dy = Math.sqrt((k2w * k2w) + (k2t_y * k2t_y));
        Log.i(TAG, "ok2t_dy= " + ok2t_dy);
        double ol2t_dy = Math.sqrt((l2w * l2w) + (l2t_y * l2t_y));
        Log.i(TAG, "ol2t_dy= " + ol2t_dy);
        double orl2t_dy = Math.sqrt(Math.abs((ok2t_dy * ok2t_dy) - (laser_high * laser_high))) - laser_depth;
        Log.i(TAG, "orl2t_dy= " + orl2t_dy);

        double cosVal_A_R = ((orl2t_dy * orl2t_dy) + (ok2t_dy * ok2t_dy) - (laser_oblique_y * laser_oblique_y)) / (2 * orl2t_dy * ok2t_dy);
        Log.i(TAG, "cosVal_A_R= " + cosVal_A_R);

        double A_R = Math.toDegrees(Math.acos(cosVal_A_R));
        Log.i(TAG, "A_R (y)= " + A_R);

        double A_K = Math.toDegrees(Math.atan(k2w / Math.abs(k2t_y)));
        Log.i(TAG, "A_K (y)= " + A_K);

        double A_L = Math.toDegrees(Math.atan(l2w / Math.abs(l2t_y)));
        Log.i(TAG, "A_L (y)= " + A_L);

        if (l2t_y > 0) {
            Log.i(TAG, "target on top  of laser : " + l2t_y);
            A_angel = (A_R + A_K) - A_L;
        } else {
            if (k2t_y < 0) {
                Log.i(TAG, "target on bottom of kibo : " + k2t_y);
                A_angel = A_K - (A_R + A_L);
            } else if (k2t_y > 0) {
                Log.i(TAG, "target on top  of kibo : " + k2t_y);
                A_angel = 180 - (A_R + A_L + A_K);
            }
        }
        Log.i(TAG, "A_angel= " + A_angel);

        double X_dis = Math.sqrt((ol2t_dy * ol2t_dy) + (orl2t_dy * orl2t_dy) - 2 * ol2t_dy * orl2t_dy * Math.cos(Math.toRadians(A_angel)));
        Log.i(TAG, "X_dis= " + X_dis);

        verticalAngle = (Math.toDegrees(Math.asin((X_dis / 2) / laser_oblique_y))) * 2;
        if (l2t_y < 0) {
            Log.i(TAG, "verticalAngle are negative");
            verticalAngle = verticalAngle * -1;
        }

        return verticalAngle;
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

}

