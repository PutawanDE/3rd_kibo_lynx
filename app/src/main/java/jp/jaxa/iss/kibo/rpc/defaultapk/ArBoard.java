package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;

import java.util.List;

public class ArBoard {
    private final double realWidth;
    private final double realHeight;

    private final List<Mat> corners;
    private final Mat ids;

    private int[] dimensionPx = new int[]{};

    public ArBoard(List<Mat> corners, Mat ids, double realWidth, double realHeight) {
        this.corners = corners;
        this.ids = ids;
        this.realWidth = realWidth;
        this.realHeight = realHeight;
    }

    public List<Mat> getCorners() {
        return corners;
    }

    public Mat getIds() {
        return ids;
    }

    public double getRealWidth() {
        return realWidth;
    }

    public double getRealHeight() {
        return realHeight;
    }

    // return top, right, bottom, left
    public int[] getDimensionPx() {
        final int markersCount = 4;
        if (dimensionPx.length != 4) {
            if (ids.rows() == markersCount && corners.size() == markersCount) {
                int topLeftX = 0, topRightX = 0, bottomLeftX = 0, bottomRightX = 0;
                int topLeftY = 0, topRightY = 0, bottomLeftY = 0, bottomRightY = 0;

                for (int i = 0; i < markersCount; i++) {
                    int id = (int) ids.get(i, 0)[0];
                    Mat markerPoints = corners.get(i);

                    if (id % 10 == 1) {
                        topRightX = (int) markerPoints.get(0, 1)[0];
                        topRightY = (int) markerPoints.get(0, 1)[1];
                    } else if (id % 10 == 2) {
                        topLeftX = (int) markerPoints.get(0, 0)[0];
                        topLeftY = (int) markerPoints.get(0, 0)[1];
                    } else if (id % 10 == 3) {
                        bottomLeftX = (int) markerPoints.get(0, 3)[0];
                        bottomLeftY = (int) markerPoints.get(0, 3)[1];
                    } else if (id % 10 == 4) {
                        bottomRightX = (int) markerPoints.get(0, 2)[0];
                        bottomRightY = (int) markerPoints.get(0, 2)[1];
                    }
                }

                int top = topRightX - topLeftX;
                int bottom = bottomRightX - bottomLeftX;
                int right = bottomRightY - topRightY;
                int left = bottomLeftY - topLeftY;
                dimensionPx = new int[]{top, right, bottom, left};
            }
        }

        return dimensionPx;
    }
}
