package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TestDetector extends OpenCVpipeline {

    private Mat hsv = new Mat();
    private Mat filtered = new Mat();
    private Mat filtered_rgba = new Mat();

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        Core.inRange(hsv, new Scalar(90, 128, 30), new Scalar(170, 255, 255), filtered);
        Imgproc.cvtColor(filtered, filtered_rgba, Imgproc.COLOR_GRAY2RGBA);
        return filtered_rgba;
    }
}
