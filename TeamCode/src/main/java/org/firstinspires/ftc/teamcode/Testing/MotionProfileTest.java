package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.DataLogger;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.ProfileGenerator;

/**
 * Created by ethan on 3/24/18.
 */
//returns motion profile output
@Autonomous
public class MotionProfileTest extends LinearOpMode {
    DataLogger data = new DataLogger("Motion profile test");
    ProfileGenerator test = new ProfileGenerator();

    int count;

    @Override
    public void runOpMode(){
        test.setInputs(0.2, 0.8, 6);

        data.addField("time");
        data.addField("output");
        data.newLine();

        waitForStart();

        resetStartTime();

        while(opModeIsActive()){
            telemetry.addData("output", test.getOutput(getRuntime()));
            telemetry.update();

            if(count == 50){ //reduces number of lines in csv file
                data.addField(getRuntime());
                data.addField(test.getOutput(getRuntime()));
                data.newLine();

                count = 0;
            }

            if(test.getOutput(getRuntime()) == 0){
                requestOpModeStop();
            }
            count++;
        }
        requestOpModeStop();
    }
}
