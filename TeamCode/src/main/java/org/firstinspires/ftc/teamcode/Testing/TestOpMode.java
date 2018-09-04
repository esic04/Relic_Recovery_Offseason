package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.TestClass;
@Autonomous
public class TestOpMode extends LinearOpMode{
    TestClass test = new TestClass(this);

    public void runOpMode(){
        waitForStart();

        while(opModeIsActive()){
            test.test();
            telemetry.update();
        }
    }
}
