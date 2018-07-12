package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TestDetector extends OpenCVpipeline {

    private Mat hsv = new Mat();
    private Mat filtered = new Mat();
    private Mat filtered_rgba = new Mat();


    public Mat processFrame(Mat rgba, Mat gray) {

        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        Core.inRange(hsv, new Scalar(18, 73, 73), new Scalar(61, 31, 255), filtered); //takes scalar input as rgb I think, not hsv
        Imgproc.cvtColor(filtered, filtered_rgba, Imgproc.COLOR_GRAY2RGBA);
        return filtered_rgba;
    }
}
