package org.firstinspires.ftc.teamcode.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by ethan on 3/9/18.
 */

public class TankDrive extends OpMode {
    DcMotor left;
    DcMotor right;
    @Override
    public void init(){
        left = hardwareMap.get(DcMotor.class, "bl");
        right = hardwareMap.get(DcMotor.class, "br");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void loop(){
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);

        telemetry.addData("bl motor pow", left.getPower());
        telemetry.addData("br motor pow", right.getPower());

    }
}
