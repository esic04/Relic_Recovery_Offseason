package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.OpenCVpipeline;
import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.TestDetector;

@Autonomous
public class OpenCVtest extends LinearOpMode{
    OpenCVpipeline opencv;

    public void runOpMode(){
        opencv = new TestDetector();
        opencv.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);

        waitForStart();
        opencv.enable();

        while(opModeIsActive()){

        }

        opencv.disable();
    }
}
