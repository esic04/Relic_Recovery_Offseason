package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DrawEdges extends OpenCVpipeline {

    private Mat rgba = new Mat();
    private Mat blurred = new Mat();
    private Mat hsv = new Mat();
    private Mat filtered = new Mat();
    private Mat edges = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Scalar lowFilter = new Scalar(100, 200, 90);
    private Scalar highFilter = new Scalar(140, 255, 255);
    private int maxAreaID;
    private double maxVal;


    public Mat processFrame(Mat rgba, Mat gray){
        this.rgba = rgba;
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        Imgproc.blur(hsv, blurred, new Size(5, 5));
        Core.inRange(blurred, lowFilter, highFilter, filtered);
        Imgproc.findContours(filtered, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int contourId = 0; contourId < contours.size(); contourId++){
            double contourArea = Imgproc.contourArea(contours.get(contourId));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxAreaID = contourId;
            }

            if(stopped){break;}

        }
        Imgproc.drawContours(rgba, contours, maxAreaID, new Scalar(255, 0, 0), 5);
        maxVal = 0; //resets max contour area after finding largest contour
        maxAreaID = 0;

        return rgba;
    }
}
