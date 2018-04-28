package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.teamcode.RobotFunctions.DataLogger;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.ProfileGenerator;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;

@Autonomous
public class ProfileRunTest extends LinearOpMode {
    Hardware robot = new Hardware();
    ProfileGenerator profile = new ProfileGenerator();
    DataLogger data = new DataLogger("motion profile");

    double leftOut, rightOut, frontLeftOut, frontRightOut, headingOut;
    double time;
    double output;
    double heading;
    double Kp = 0.23; //kp 0.3, ki 0.00004 kd 0 works sort of
    double Ki = 0.000115; //tuning constants
    double Kd = 0.0000000;//5;
    double headingP = 0.01; //kp 0.003, ki 0.00000012, kd 0.000000005 works sort of
    double headingI = 0.000004;
    double headingD = 0.0005;

    PID pid1 = new PID(Kp, Ki, Kd, 0.4, -1, 1);
    PID pid2 = new PID(Kp, Ki, Kd, 0.4, -1, 1);
    PID pid3 = new PID(Kp, Ki, Kd, 0.4, -1, 1);
    PID pid4 = new PID(Kp, Ki, Kd, 0.4, -1, 1); //1= back left, 2 = front left, 3 = back right, 4 = front right
    PID headingPID = new PID(headingP, headingI, headingD, 0.2, -0.6, 0.6);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        profile.setInputs(0.8, 1.3, 4);

        data.addField("time");
        data.addField("motor speed");
        data.addField("profile output");
        data.newLine();

        robot.driveTrain.resetEncoders();
        robot.driveTrain.setMode(DriveTrain.motor_mode.run_with_encoder);

        waitForStart();

        resetStartTime(); //might be needed?
        while(opModeIsActive()){

            time = getRuntime();

            heading = (robot.sensors.getHeading() + 90) % 360;

            output = profile.getOutput(time);

            headingOut = headingPID.getOutput(heading, 90);

            leftOut = pid1.getOutput(robot.driveTrain.leftSpeed(), output) + leftOut - headingOut;  //for velocity pid, adds pid out to previous output for better control
            rightOut = pid3.getOutput(robot.driveTrain.rightSpeed(), output) + rightOut + headingOut;
            frontLeftOut = pid2.getOutput(robot.driveTrain.frontLeftSpeed(), output) + frontLeftOut - headingOut;
            frontRightOut = pid4.getOutput(robot.driveTrain.frontRightSpeed(), output) + frontRightOut + headingOut;

            robot.driveTrain.left.setPower(leftOut);
            robot.driveTrain.right.setPower(rightOut);
            robot.driveTrain.frontLeft.setPower(frontLeftOut);
            robot.driveTrain.frontRight.setPower(frontRightOut);

            data.addField(time);
            data.addField((robot.driveTrain.leftSpeed() + robot.driveTrain.rightSpeed() + robot.driveTrain.frontLeftSpeed() + robot.driveTrain.frontRightSpeed()) / 4);
            data.addField(output);
            data.newLine();

            telemetry.addData("profile output", output);
            telemetry.addData("left actual speed", robot.driveTrain.leftSpeed());
            telemetry.addData("right actual speed", robot.driveTrain.rightSpeed());
            telemetry.addData("heading", heading);
            telemetry.addData("left pow", leftOut);
            telemetry.update();

        }


    }
}
