package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

public class Target {
    private double xc; // x coordinate in camera frame
    private double yc; // y coordinate in camera frame

    public Target(Mat undistortedImg, double undistortedCropPosX, double undistortedCropPosY,
                  double[] cameraMatrix, double astrobeeToWallDist_z) {
        String TAG = "Target";

        final double fx = cameraMatrix[0];
        final double fy = cameraMatrix[4];
        final double cx = cameraMatrix[2];
        final double cy = cameraMatrix[5];

        final double navXOffset = -0.0422;
        final double navYOffset = 0.0826;

        double[] targetPosImg = findCircle(undistortedImg, undistortedCropPosX, undistortedCropPosY);
        double shiftedTargetPosImgX = targetPosImg[0] - cx;
        double shiftedTargetPosImgY = targetPosImg[1] - cy;

        this.xc = (shiftedTargetPosImgX * astrobeeToWallDist_z / fx) + navXOffset;
        this.yc = -(shiftedTargetPosImgY * astrobeeToWallDist_z / fy) + navYOffset;

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
