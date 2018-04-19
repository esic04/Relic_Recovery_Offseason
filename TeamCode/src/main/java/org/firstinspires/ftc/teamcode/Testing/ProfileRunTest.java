package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.DataLogger;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.ProfileGenerator;
@Autonomous
public class ProfileRunTest extends LinearOpMode {
    Hardware robot = new Hardware();
    ProfileGenerator profile = new ProfileGenerator();
    DataLogger data = new DataLogger("motion profile");

    double leftSpeed, rightSpeed, frontLeftSpeed, frontRightSpeed;
    double time;
    double profileOut;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.driveTrain.SetBrake();

        profile.setInputs(1.2, 1.8, 5);

        data.addField("time");
        data.addField("motor speed");
        data.addField("profile output");
        data.newLine();

        robot.driveTrain.resetEncoders();

        waitForStart();

        resetStartTime(); //might be needed?
        while(opModeIsActive()){

            time = getRuntime();

            profileOut = profile.getOutput(time);

            robot.driveTrain.runProfile(profileOut);

            data.addField(time);
            data.addField((robot.driveTrain.leftSpeed() + robot.driveTrain.rightSpeed() + robot.driveTrain.frontLeftSpeed() + robot.driveTrain.frontRightSpeed()) / 4);
            data.addField(profileOut);
            data.newLine();

            telemetry.addData("profile output", profileOut);
            telemetry.addData("left actual speed", leftSpeed);
            telemetry.update();

        }


    }
}
