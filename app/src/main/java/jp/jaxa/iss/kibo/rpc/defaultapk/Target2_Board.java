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
    private final int detectedIdsCount;

    public Target2_Board(List<Mat> corners, Mat ids) {
        this.ids = ids;
        this.corners = corners;
        detectedIdsCount = ids.rows();
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
        return detectedIdsCount == 4;
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

        final double[] marker12_origin = {0, 0, 0};
        final double[] marker11_origin = {target2_marker_x_dist, 0, 0};
        final double[] marker14_origin = {target2_marker_x_dist, target2_marker_y_dist, 0};
        final double[] marker13_origin = {0, target2_marker_y_dist, 0};

        List<Mat> markerObjPoints = new ArrayList<>();

        for (int i = 0; i < detectedIdsCount; i++) {
            int id = (int) ids.get(i, 0)[0];

            if (id == 11) markerObjPoints.add(setMarkerCornersObjPoint(marker11_origin));
            else if (id == 12) markerObjPoints.add(setMarkerCornersObjPoint(marker12_origin));
            else if (id == 13) markerObjPoints.add(setMarkerCornersObjPoint(marker13_origin));
            else if (id == 14) markerObjPoints.add(setMarkerCornersObjPoint(marker14_origin));
        }
        return markerObjPoints;
    }
    
    private Mat setMarkerCornersObjPoint(double[] markerOrigin) {
        Point3 topLeft = new Point3(markerOrigin);
        Point3 topRight = new Point3(markerOrigin[0] + MARKER_LENGTH,
                markerOrigin[1], markerOrigin[2]);
        Point3 bottomRight = new Point3(markerOrigin[0] + MARKER_LENGTH,
                markerOrigin[1] + MARKER_LENGTH, markerOrigin[2]);
        Point3 bottomLeft = new Point3(markerOrigin[0],
                markerOrigin[1] + MARKER_LENGTH, markerOrigin[2]);

        return new MatOfPoint3f(topLeft, topRight, bottomRight,
                bottomLeft);
    }
}
