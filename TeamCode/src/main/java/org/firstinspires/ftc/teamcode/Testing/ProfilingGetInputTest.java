package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.ProfileGenerator;
@TeleOp
public class ProfilingGetInputTest extends LinearOpMode {
    ProfileGenerator test = new ProfileGenerator();
    @Override
    public void runOpMode(){
        test.setInputs(1,1,1);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("accel", test.getParameters(0));
            telemetry.addData("speed", test.getParameters(1));
            telemetry.addData("dist", test.getParameters(2));
            telemetry.update();
        }

    }

}
