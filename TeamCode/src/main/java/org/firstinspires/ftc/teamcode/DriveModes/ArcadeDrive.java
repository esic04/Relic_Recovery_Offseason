package org.firstinspires.ftc.teamcode.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

/**
 * Created by ethan on 3/9/18.
 */
@TeleOp
public class ArcadeDrive extends OpMode {
    TankHardware robot = new TankHardware();
    @Override
    public void init(){
        robot.init(hardwareMap, this);

    }
    @Override
    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);

        telemetry.addData("left pow", robot.driveTrain.left.getPower());
        telemetry.addData("right pow", robot.driveTrain.right.getPower());
        telemetry.addData("heading", robot.sensors.getHeading());

    }
}
