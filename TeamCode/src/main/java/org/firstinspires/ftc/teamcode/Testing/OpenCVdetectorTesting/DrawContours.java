package org.firstinspires.ftc.teamcode.Testing.OpenCVdetectorTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision.Detectors.DrawEdges;


@Autonomous
public class DrawContours extends LinearOpMode {
    DrawEdges detector;

    public void runOpMode(){
        detector = new DrawEdges();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);

        waitForStart();
        detector.enable();

        while(opModeIsActive()){

        }

        detector.disable();

    }
}
