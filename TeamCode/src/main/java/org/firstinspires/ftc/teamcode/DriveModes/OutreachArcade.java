package org.firstinspires.ftc.teamcode.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

@TeleOp
public class OutreachArcade extends OpMode {
    TankHardware robot = new TankHardware();

    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.driveTrain.br.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.driveTrain.fr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        robot.driveTrain.bl.setPower(-gamepad1.left_stick_y - (0.75 * gamepad1.left_stick_x));
        robot.driveTrain.br.setPower(-gamepad1.left_stick_y + (0.75 * gamepad1.left_stick_x));
        robot.driveTrain.fl.setPower(-gamepad1.left_stick_y - (0.75 * gamepad1.left_stick_x));
        robot.driveTrain.fr.setPower(-gamepad1.left_stick_y + (0.75 * gamepad1.left_stick_x));
    }
}
