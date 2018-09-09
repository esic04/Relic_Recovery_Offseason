package org.firstinspires.ftc.teamcode.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.sqrt;

/**
 * Created by ethan on 3/9/18.
 */
@TeleOp
public class SqrtDrive extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor frontLeft;
    DcMotor frontRight;
    @Override
    public void init(){
        left = hardwareMap.get(DcMotor.class, "bl");
        right = hardwareMap.get(DcMotor.class, "br");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");


        right.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void loop(){
        left.setPower(sqrt(gamepad1.left_stick_y - gamepad1.left_stick_x));
        right.setPower(sqrt(gamepad1.left_stick_y + gamepad1.left_stick_x));
        frontLeft.setPower(sqrt(gamepad1.left_stick_y - gamepad1.left_stick_x));
        frontRight.setPower(sqrt(gamepad1.left_stick_y + gamepad1.left_stick_x));

        telemetry.addData("bl motor pow", left.getPower());
        telemetry.addData("br motor pow", right.getPower());

    }
}
