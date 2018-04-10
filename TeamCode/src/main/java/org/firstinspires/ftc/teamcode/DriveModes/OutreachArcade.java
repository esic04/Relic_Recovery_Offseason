package org.firstinspires.ftc.teamcode.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
@TeleOp
public class OutreachArcade extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.driveTrain.right.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.driveTrain.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        robot.driveTrain.left.setPower(-gamepad1.left_stick_y - (0.75 * gamepad1.left_stick_x));
        robot.driveTrain.right.setPower(-gamepad1.left_stick_y + (0.75 * gamepad1.left_stick_x));
        robot.driveTrain.frontLeft.setPower(-gamepad1.left_stick_y - (0.75 * gamepad1.left_stick_x));
        robot.driveTrain.frontRight.setPower(-gamepad1.left_stick_y + (0.75 * gamepad1.left_stick_x));
    }
}
