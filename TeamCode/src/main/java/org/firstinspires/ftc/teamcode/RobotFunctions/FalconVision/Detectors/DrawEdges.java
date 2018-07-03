package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class DrawEdges extends OpenCVpipeline {

    private Mat gray = new Mat();
    private Mat blured = new Mat();
    private Mat edges = new Mat();

    public Mat processFrame(Mat rgba, Mat gray){
        this.gray = gray;
        Imgproc.GaussianBlur(gray, blured, new Size(5, 5), 0, 0);
        Imgproc.Canny(blured, edges, 50, 150);

        return edges;
    }
}
