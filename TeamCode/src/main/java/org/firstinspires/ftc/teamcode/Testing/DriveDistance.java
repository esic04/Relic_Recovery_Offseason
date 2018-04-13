package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;

public class DriveDistance extends OpMode {
    Hardware robot = new Hardware();
    Calculators cal = new Calculators();

    @Override
    public void init(){
        robot.init(hardwareMap);
    }


    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);

        telemetry.addData("left dist", cal.Encoder2Ft(robot.driveTrain.left.getCurrentPosition()));
        telemetry.addData("right dist", cal.Encoder2Ft(robot.driveTrain.right.getCurrentPosition()));
    }
}
