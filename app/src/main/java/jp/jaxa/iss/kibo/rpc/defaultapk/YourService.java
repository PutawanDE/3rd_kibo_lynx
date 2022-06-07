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

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 5;

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

    final double target2_board_width = 0.275;
    final double target2_board_height = 0.133;
    final double MARKER_LENGTH = 0.05;

    long debug_Timestart = 0;
    private boolean FAIL_to_Find_TARGET;

    @Override
    protected void runPlan1() {
        debug_Timestart = System.currentTimeMillis();
        api.startMission();

        // move to point A
        Quaternion quaternionAtB = new Quaternion(0f, 0.707f, 0f, 0.707f);
        move_to(10.71, -7.7, 4.48, quaternionAtB);
        api.reportPoint1Arrival();

        // shoot laser
        move_to(10.71, -7.811971, 4.48, quaternionAtB);
        api.laserControl(true);
        api.takeTarget1Snapshot();


        // move to point B
        Point B = new Point(11.2746, -9.92284, 5.29881);
        quaternionAtB = new Quaternion(0f, 0f, -0.707f, 0.707f);
        move_to(10.876214285714285, -8.5, 4.97063, quaternionAtB);
        move_to(11.0067, -9.44819, 5.186407142857143, quaternionAtB);
        move_to(B, quaternionAtB);

        // Wait for camera delay or Astrobee to stop moving due to inertia
        try {
            Thread.sleep(6000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double astrobeeToWallDist_z = estimateAstrobeeToWallDist_z(api.getMatNavCam());
        final int cropPosX = 500;
        final int cropPosY = 500;
        Mat croppedImg = new Mat(api.getMatNavCam(), new Rect(cropPosX, cropPosY, 330, 200));
        Mat targetImg = undistortCroppedImg(croppedImg, cropPosX, cropPosY);
        Target target = new Target(targetImg, cropPosX, cropPosY, CAM_MATSIM, astrobeeToWallDist_z);
        api.saveMatImage(targetImg, "ud_target_img.png");

        final double xAngle = -computeXAngle(astrobeeToWallDist_z, target.getY());
        final double yAngle = -computeYAngle(astrobeeToWallDist_z, target.getX());

        Quaternion relativeQ = eulerAngleToQuaternion(xAngle, 0, yAngle);
        Quaternion absoluteQ = mutiplyQuaternion(relativeQ, quaternionAtB);

        String TAG = "Rotation";
        Log.i(TAG, "relativeQ = " + relativeQ.toString());
        Log.i(TAG, "absoluteQ = " + absoluteQ.toString());

        // try rotating to wanted angle and shoot laser
        Quaternion initialQuaternion = new Quaternion(0, 0, -0.6427876f, 0.7660444f);
        move_to(B, initialQuaternion);; // reset astrobee
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
                } catch (Exception ignored) {}
                api.laserControl(true);
                break;
            }
            else if (retry == LOOP_MAX) {
                // last resort shooting
                try {
                    Thread.sleep(500);
                } catch (Exception ignored) {}
                api.laserControl(true);
                break;
            }

            move_to(B, initialQuaternion);; // reset astrobee
            retry++;
        }

        api.takeTarget2Snapshot();

        // move to goal
        move_to(11.0067, -9.44819, 5.1722, quaternionAtB);
        move_to(11.073773469387755, -8.5, 4.97063, quaternionAtB);
        move_to(11.2746, -7.89178, 4.96538, quaternionAtB);

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

    private double estimateAstrobeeToWallDist_z(Mat ar_img) {
        final Dictionary ARUCO_DICT = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        final String TAG = "estimateAstrobeeToWallDist_z";
        api.saveMatImage(ar_img, "InputBoardToEstimate.png");

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        // TODO find the best fixedAstrobeeToWallDist
        double fixedAstrobeeToWallDist_z = 0.65;

        try {
            Log.i(TAG, "Read AR for ARUCO board pose estimation");
            ArBoard detectedBoard = readAR(ar_img);
            Log.i(TAG, "Reading AR DONE########");
            if (!detectedBoard.isBoardComplete()) {
                Log.e(TAG, "Fail to estimate board pose, not all 4 markers are found");
                Log.i(TAG, "Due to failure, will return fixed AstrobeeToWallDist_z= " + fixedAstrobeeToWallDist_z);
                return fixedAstrobeeToWallDist_z;
            }

            List<Mat> corners = detectedBoard.getCorners();
            Mat detectedIDs = detectedBoard.getIds();

            Mat tvec = new Mat();
            Mat rvec = new Mat();

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

            Aruco.drawAxis(ar_img, cameraMat, distCoeffs, rvec, tvec, (float) MARKER_LENGTH);
            api.saveMatImage(ar_img, "ArucoBoardPose.png");
            double astrobeeToWallDist = tvec.get(2, 0)[0] + CAM_POS[0];
            Log.i(TAG, "Astrobee2WallDist_z=" + astrobeeToWallDist);
            return astrobeeToWallDist;
        } catch (Exception e) {
            Log.e(TAG, "Fail to estimate board pose", e);
            Log.i(TAG, "Due to failure, returning fixed Astrobee2WallDist_z= " + fixedAstrobeeToWallDist_z);
            return fixedAstrobeeToWallDist_z;
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

            Mat detectedMarkerImg = new Mat();
            Imgproc.cvtColor(img, detectedMarkerImg, Imgproc.COLOR_GRAY2RGB);
            Aruco.drawDetectedMarkers(detectedMarkerImg, corners);
            api.saveMatImage(detectedMarkerImg, "ArucoDetectedMarker.png");
        } catch (Exception e) {
            Log.e(TAG, "Fail reading AR tags", e);
        }

        return new ArBoard(corners, detectedIDs, target2_board_width, target2_board_height);
    }

    private List<Mat> initializeTarget2AR_points() {
        final double target2_marker_x_dist = 0.225;
        final double target2_marker_y_dist = 0.083;

        final double[] marker12_point = {0, 0, 0};
        final double[] marker11_point = {target2_marker_x_dist, 0, 0};
        final double[] marker14_point = {target2_marker_x_dist, target2_marker_y_dist, 0};
        final double[] marker13_point = {0, target2_marker_y_dist, 0};

        final double[][] markersPoint = {
                marker12_point, marker11_point, marker14_point, marker13_point
        };

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

    private Mat undistortCroppedImg(Mat img, int x, int y) {
        // change principal point of CAM_MATSIM
        double[] NEW_CAM_MAT = new double[9];
        System.arraycopy(CAM_MATSIM, 0, NEW_CAM_MAT, 0, 9);
        NEW_CAM_MAT[2] -= x;
        NEW_CAM_MAT[5] -= y;

        Mat ud_img = new Mat(img.rows(), img.cols(), img.type());
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, NEW_CAM_MAT);
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
        Log.i(TAG, "turn_angle = " + Math.toDegrees(angle1 - angle2) + " deg");

        return angle1 - angle2;
    }

    // left angle
    private double computeYAngle(double l2t, double xc) {
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
        Log.i(TAG, "turn_angle = " + Math.toDegrees(angle1 - angle2) + " deg");

        return angle1 - angle2;
    }

    // https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
        double qx = Math.sin(xAngle/2) * Math.cos(yAngle/2) * Math.cos(zAngle/2) -
                Math.cos(xAngle/2) * Math.sin(yAngle/2) * Math.sin(zAngle/2);
        double qy = Math.cos(xAngle/2) * Math.sin(yAngle/2) * Math.cos(zAngle/2) +
                Math.sin(xAngle/2) * Math.cos(yAngle/2) * Math.sin(zAngle/2);
        double qz = Math.cos(xAngle/2) * Math.cos(yAngle/2) * Math.sin(zAngle/2) -
                Math.sin(xAngle/2) * Math.sin(yAngle/2) * Math.cos(zAngle/2);
        double qw = Math.cos(xAngle/2) * Math.cos(yAngle/2) * Math.cos(zAngle/2) +
                Math.sin(xAngle/2) * Math.sin(yAngle/2) * Math.sin(zAngle/2);

        return new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
    }

    private Quaternion mutiplyQuaternion(Quaternion q1, Quaternion q2) {
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

