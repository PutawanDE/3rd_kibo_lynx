package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

public class Target2_Board {
    public final double MARKER_LENGTH = 0.05;

    private final double realWidth = 0.275;
    private final double realHeight = 0.133;

    private final List<Mat> corners;
    private final Mat ids;
    private final List<Mat> markerObjPoints;

    public Target2_Board(List<Mat> corners, Mat ids) {
        this.corners = corners;
        this.ids = ids;
        this.markerObjPoints = setObjPoints();
    }

    public List<Mat> getCorners() {
        return corners;
    }

    public Mat getIds() {
        return ids;
    }

    public List<Mat> getMarkerObjPoints() {
        return markerObjPoints;
    }

    public boolean isBoardComplete() {
        return corners.size() == 4;
    }

    public double getRealWidth() {
        return realWidth;
    }

    public double getRealHeight() {
        return realHeight;
    }

    private List<Mat> setObjPoints() {
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
}
