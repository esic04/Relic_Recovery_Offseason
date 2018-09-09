package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotFunctions.DataLogger;
import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.ProfileGenerator;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;

@Autonomous
public class ProfileRunTest extends LinearOpMode {
    TankHardware robot = new TankHardware();
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
    double angularSpeed;
    double gearRatio = 2.0/3;
    double wheelr = 2;

    PID pid1 = new PID(Kp, Ki, Kd, 0.4, -1, 1);
    PID pid2 = new PID(Kp, Ki, Kd, 0.4, -1, 1);
    PID pid3 = new PID(Kp, Ki, Kd, 0.4, -1, 1);
    PID pid4 = new PID(Kp, Ki, Kd, 0.4, -1, 1); //1= back bl, 2 = front bl, 3 = back br, 4 = front br
    PID headingPID = new PID(headingP, headingI, headingD, 0.2, -0.6, 0.6);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);

        profile.setInputs(1, 1.6, 4);

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

            output = profile.getOutput(time) * 12; //convert ft/sec to in/sec

            /* headingOut = headingPID.getOutput(heading, 90);

            leftOut = pid1.getOutput(robot.driveTrain.speed(DriveTrain.motor.bl), output) + leftOut - headingOut;  //for velocity pid, adds pid out to previous output for better control
            rightOut = pid3.getOutput(robot.driveTrain.speed(DriveTrain.motor.br), output) + rightOut + headingOut;
            frontLeftOut = pid2.getOutput(robot.driveTrain.speed(DriveTrain.motor.fl), output) + frontLeftOut - headingOut;
            frontRightOut = pid4.getOutput(robot.driveTrain.speed(DriveTrain.motor.fr), output) + frontRightOut + headingOut;

            robot.driveTrain.bl.setPower(leftOut);
            robot.driveTrain.br.setPower(rightOut);
            robot.driveTrain.fl.setPower(frontLeftOut);
            robot.driveTrain.fr.setPower(frontRightOut);

            */

            data.addField(time);
            data.addField(((robot.driveTrain.speed(DriveTrain.motor.bl) + robot.driveTrain.speed(DriveTrain.motor.br) + robot.driveTrain.speed(DriveTrain.motor.fl) + robot.driveTrain.speed(DriveTrain.motor.fr)) / 4) * 12);
            data.addField(output);
            data.newLine();


            angularSpeed = output / wheelr / gearRatio;

            robot.driveTrain.bl.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.br.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.fl.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.fr.setVelocity(angularSpeed, AngleUnit.RADIANS);

            telemetry.addData("profile output", output);
            telemetry.addData("bl actual speed", robot.driveTrain.speed(DriveTrain.motor.bl) * 12);
            telemetry.addData("br actual speed", robot.driveTrain.speed(DriveTrain.motor.br) * 12);
            telemetry.addData("heading", heading);
            telemetry.addData("bl pow", robot.driveTrain.bl.getPower());
            telemetry.update();

        }


    }
}
