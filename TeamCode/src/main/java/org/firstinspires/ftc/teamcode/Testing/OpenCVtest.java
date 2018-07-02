package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.visionStuff.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.visionStuff.OpenCV;
import org.firstinspires.ftc.teamcode.RobotFunctions.visionStuff.TestDetector;

@Autonomous
public class OpenCVtest extends LinearOpMode{
    OpenCV opencv;

    public void runOpMode(){
        opencv = new TestDetector();
        opencv.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);

        waitForStart();

        while(opModeIsActive()){
            opencv.enable();
        }


    }
}
