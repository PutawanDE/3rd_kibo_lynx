package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;

import java.util.List;

public class ArBoard {
    private final double realWidth;
    private final double realHeight;

    private final List<Mat> corners;
    private final Mat ids;

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

    public boolean isBoardComplete() {
        return corners.size() == 4;
    }

    public double getRealWidth() {
        return realWidth;
    }

    public double getRealHeight() {
        return realHeight;
    }
}
