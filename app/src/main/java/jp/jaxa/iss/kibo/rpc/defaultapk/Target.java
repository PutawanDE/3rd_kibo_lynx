package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

public class Target {
    private double xc; // x coordinate in camera frame
    private double yc; // y coordinate in camera frame
    private Mat CenterTarget_nav;
    private Mat CenterTarget_crop;

    public Target(Mat MatNavCam, Mat undistortedImg, double undistortedCropPosX, double undistortedCropPosY,
                  double[] cameraMatrix, double astrobeeToWallDist_z) {
        String TAG = "Target";

        double xc_offset = -0.023;
        double yc_offset = -0.00343;

        final double fx = cameraMatrix[0];
        final double fy = cameraMatrix[4];
        final double cx = cameraMatrix[2];
        final double cy = cameraMatrix[5];

        final double cam_width   = -0.0422;
        final double cam_high   = 0.0826;

        double[] targetPosImg = findCircle(undistortedImg, undistortedCropPosX, undistortedCropPosY);
        double shiftedTargetPosImgX = targetPosImg[0] - cx;
        double shiftedTargetPosImgY = targetPosImg[1] - cy;

        Mat ArucoDetectedCenter_crop = undistortedImg.clone();
        org.opencv.core.Point center_crop = new org.opencv.core.Point( targetPosImg[0] - undistortedCropPosX   , targetPosImg[1] - undistortedCropPosY);
        Imgproc.circle(ArucoDetectedCenter_crop, center_crop, 1, new Scalar(255, 0, 255), 3, 8, 0);
        CenterTarget_crop =  (ArucoDetectedCenter_crop);

        Mat ArucoDetectedCenter_nav = MatNavCam.clone();
        org.opencv.core.Point center_nav = new org.opencv.core.Point( targetPosImg[0], targetPosImg[1]);
        Imgproc.circle(ArucoDetectedCenter_nav, center_nav, 1, new Scalar(255, 0, 255), 3, 8, 0);
        CenterTarget_nav =  (ArucoDetectedCenter_nav);

        this.xc = (shiftedTargetPosImgX * astrobeeToWallDist_z / fx) + cam_width + xc_offset  ;
        this.yc = -(shiftedTargetPosImgY * astrobeeToWallDist_z / fy) + cam_high + yc_offset ;

        Log.i(TAG, "targetPosImg = " + Arrays.toString(targetPosImg));
        Log.i(TAG, "shiftedTargetPosImgX= " + shiftedTargetPosImgX);
        Log.i(TAG, "shiftedTargetPosImgY= " + shiftedTargetPosImgY);
        Log.i(TAG, "xc, yc =" + xc + ", " + yc);
    }

    public double getX() {
        return xc;
    }

    public double getY() {
        return yc;
    }

    public Mat getCenterTarget_crop(){
        return  CenterTarget_crop;
    }
    public Mat getCenterTarget_nav(){
        return  CenterTarget_nav;
    }

    private double[] findCircle(Mat ud_img, double x, double y) {
        Mat circle = new Mat();
        Imgproc.HoughCircles(ud_img, circle, Imgproc.HOUGH_GRADIENT, 1, 300, 50, 30, 40, 50);

        if (circle.empty())
            // TODO: find best fixed point to return
            return null;

        double[] point = {circle.get(0, 0)[0] + x, circle.get(0, 0)[1] + y};
        return point;
    }
}
