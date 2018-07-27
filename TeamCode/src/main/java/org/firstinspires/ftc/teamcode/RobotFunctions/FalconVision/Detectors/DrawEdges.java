package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
    private Mat edges = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Scalar lowFilterHigh = new Scalar(15, 255, 255); //for two different filters, because the range for orange is around 0 and around 360 (hue)
    private Scalar lowFilterLow = new Scalar(0, 20, 120);
    private Scalar highFilterHigh = new Scalar(180, 255, 255);
    private Scalar highFilterLow = new Scalar(175, 20, 120);
    private MatOfPoint pts = new MatOfPoint();
    private Rect boundingRect = new Rect();
    private int maxAreaID;
    private double maxVal;
    private double maxArea;
    double contourArea;


    public Mat processFrame(Mat rgba, Mat gray){
        this.rgba = rgba;
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        Imgproc.blur(hsv, blurred, new Size(15, 15));
        Core.inRange(blurred, lowFilterLow, lowFilterHigh, filteredLow);
        Core.inRange(blurred, highFilterLow, highFilterHigh, filteredHigh);
        Core.add(filteredLow, filteredHigh, filteredCombined);
        //Imgproc.findContours(filteredCombined, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Core.findNonZero(filteredCombined, pts);

        boundingRect = Imgproc.boundingRect(pts);
        Imgproc.rectangle(rgba, boundingRect.tl(), boundingRect.br(), new Scalar(0,0,255), 5);


        //Imgproc.drawContours(rgba, contours, maxAreaID, new Scalar(255, 0, 0), 5);
        maxVal = 0; //resets max contour area after finding largest contour

        return rgba;
    }

    public double getMaxArea(){return maxArea;}
}
