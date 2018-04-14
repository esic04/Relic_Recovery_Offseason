package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.Pose;

@TeleOp
public class GetPosition extends OpMode {
    Hardware robot = new Hardware();
    Pose position;
    Calculators cal = new Calculators();


    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);

        position = robot.driveTrain.GetPose(robot.sensors.getHeading());

        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("heading", position.getHeading());

        telemetry.update();



    }
}
