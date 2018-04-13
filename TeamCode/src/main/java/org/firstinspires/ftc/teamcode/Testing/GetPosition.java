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

    double Vr, Vl, Vrx, Vry, omegaR;
    double hdg;
    double Vwx, Vwy;
    double Xkp, Ykp, thetaK;
    double currTime, preTime, deltaTime;
    double width;

    @Override
    public void init(){
        robot.init(hardwareMap);
        width = 16.25 / 12;
    }

    @Override
    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);

        hdg = Math.toRadians(robot.sensors.getHeading());

        currTime = System.currentTimeMillis() / 1000.0;

        Vr = (robot.driveTrain.rightSpeed() + robot.driveTrain.frontRightSpeed()) / 2;
        Vl = (robot.driveTrain.leftSpeed() + robot.driveTrain.frontLeftSpeed()) / 2;

        deltaTime = currTime - preTime;

        Vrx = (Vr + Vl) / 2;
        Vry = 0; // makes robot non holonomic
        omegaR = (Vr - Vl) / width;

        Vwx = Vrx * Math.cos(hdg) - Vry * Math.sin(hdg);
        Vwy = Vrx * Math.sin(hdg) + Vry * Math.cos(hdg);

        Xkp = Xkp + Vwx * deltaTime;
        Ykp = Ykp + Vwy * deltaTime;

        preTime = currTime;

        telemetry.addData("x", Xkp);
        telemetry.addData("y", Ykp);
        telemetry.addData("right speed", robot.driveTrain.rightSpeed());
        telemetry.addData("left speed", robot.driveTrain.leftSpeed());
        telemetry.update();



    }
}
