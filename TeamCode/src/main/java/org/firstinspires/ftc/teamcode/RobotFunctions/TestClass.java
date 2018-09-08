package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is an example of how to pass an opmode to a non opmode class, and how to send a telemetry message from the non opmode class
 *
 * @author ethan
 */

public class TestClass {
    LinearOpMode linOpMode;

    public TestClass(LinearOpMode linOpMode){this.linOpMode = linOpMode;}

    public void test(){
        while(linOpMode.opModeIsActive()){
            linOpMode.telemetry.addData("testing", "hopefuly works");
            linOpMode.telemetry.update();
        }
    }


}
