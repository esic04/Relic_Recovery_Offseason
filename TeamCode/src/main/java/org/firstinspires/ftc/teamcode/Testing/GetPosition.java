package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

@TeleOp
public class GetPosition extends OpMode {
    Hardware robot = new Hardware();
    Point position = new Point(0, 0);
    Calculators cal = new Calculators();
    Point origin = new Point(0, 0);


    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);

        position = robot.driveTrain.GetPosition(robot.sensors.getHeading());

        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("distance from start", cal.PointDistance(origin, robot.driveTrain.GetPosition(robot.sensors.getHeading())));

        telemetry.update();



    }
}
