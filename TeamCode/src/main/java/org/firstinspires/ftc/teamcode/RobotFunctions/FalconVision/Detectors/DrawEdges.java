package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DrawEdges extends OpenCVpipeline {

    private Mat grayScale = new Mat();
    private Mat blurred = new Mat();
    private Mat edges = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private int maxAreaID;
    private double maxVal;


    public Mat processFrame(Mat rgba, Mat gray){

        Imgproc.cvtColor(rgba, grayScale, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(grayScale, blurred, new Size(5, 5), 0, 0);
        Imgproc.Canny(blurred, edges, 20, 150);
        Imgproc.findContours(blurred, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int contourId = 0; contourId < contours.size(); contourId++){
            double contourArea = Imgproc.contourArea(contours.get(contourId));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxAreaID = contourId;
            }
        }

        Imgproc.drawContours(rgba, contours, maxAreaID, new Scalar(0, 102, 255), 5);

        return rgba;
    }
}
