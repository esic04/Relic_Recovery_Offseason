package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestClass {
    LinearOpMode opmode;

    public TestClass(LinearOpMode opmode){
        this.opmode = opmode;
    }

    public void test(){
        while(opmode.opModeIsActive()){
            opmode.telemetry.addData("testing", "hopefuly works");
            opmode.telemetry.update();
        }
    }


}
