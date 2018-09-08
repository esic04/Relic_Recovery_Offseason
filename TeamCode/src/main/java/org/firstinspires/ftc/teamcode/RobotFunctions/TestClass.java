package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is an example of how to pass an opmode to a non opmode class, and how to send a telemetry message from the non opmode class
 *
 * @author ethan
 */

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
