package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Target {
    private double xc; // x coordinate
    private double yc; // y coordinate

    public Target(Mat ud_img, int cX, int cY) {
        final double navXOffset = -0.0422;
        final double navYOffset = 0.0826;
        double meterPx = computeMeterPx(ud_img);
        double[] target = findCircle(ud_img, cX, cY);

        double[] center = {635.434258, 500.335102}; // principal point
        this.xc = ((target[0] - center[0]) * meterPx) + navXOffset;
        this.yc = -((target[1] - center[1]) * meterPx) + navYOffset;

        String TAG = "Target";
        Log.i(TAG, "meterPerPixel = " + meterPx);
        Log.i(TAG, "target = " + Arrays.toString(target));
        Log.i(TAG, "xc, yc =" + xc + ", " + yc);
    }

    public double getX() {
        return xc;
    }

    public double getY() {
        return yc;
    }

    private double distance(double x0, double y0, double x1, double y1) {
        return Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));
    }

    private double computeMeterPx(Mat ud_img) {
        String TAG = "computeMeterPx";

        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        final Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        int counter = 0;
        while (ids.rows() < 4 && counter < 3) { // 3 try until find all 4 ids
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
            double sAvg = (s1 + s2 + s3 + s4) / 4;
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
