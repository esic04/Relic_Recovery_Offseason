package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DrawEdges extends OpenCVpipeline {

    private Mat rgba = new Mat();
    private Mat blurred = new Mat();
    private Mat hsv = new Mat();
    private Mat filteredLow = new Mat();
    private Mat filteredHigh = new Mat();
    private Mat filteredCombined = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Scalar lowFilterHigh = new Scalar(15, 255, 255); //for two different filters, because the range for orange is around 0 and around 360 (hue)
    private Scalar lowFilterLow = new Scalar(0, 100, 80);
    private Scalar highFilterHigh = new Scalar(180, 255, 255);
    private Scalar highFilterLow = new Scalar(175, 100, 80);
    MatOfPoint2f approxCurve = new MatOfPoint2f();
    double rectArea;


    public Mat processFrame(Mat rgba, Mat gray){
        this.rgba = rgba;
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        Imgproc.blur(hsv, blurred, new Size(5, 5));
        Core.inRange(blurred, lowFilterLow, lowFilterHigh, filteredLow);
        Core.inRange(blurred, highFilterLow, highFilterHigh, filteredHigh);
        Core.add(filteredLow, filteredHigh, filteredCombined);
        Imgproc.findContours(filteredCombined, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect[] rect = new Rect[contours.size()];

        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint2f contour2f = new MatOfPoint2f( contours.get(i).toArray());
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());
            rect[i] = Imgproc.boundingRect(points);
        }

        for (int i = 0; i < contours.size(); i++){

            Imgproc.rectangle(rgba, rect[i].tl(), rect[i].br(), new Scalar(0, 255, 255), 5);
        }

        contours.clear();

        return rgba;
    }

    public double getMaxArea(){return rectArea;}
}
