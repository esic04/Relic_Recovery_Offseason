package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by ethan on 3/6/18.
 */
@Autonomous
public class encoderTEst extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    DcMotor frontLeft;
    DcMotor frontRight;

    @Override
    public void runOpMode(){
        left = hardwareMap.get(DcMotor.class, "bl");
        right = hardwareMap.get(DcMotor.class, "br");
        frontLeft = hardwareMap.get(DcMotor.class,  "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");



        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setTargetPosition(left.getCurrentPosition() + 1120);
        right.setTargetPosition(right.getCurrentPosition() +1120);
        frontLeft.setTargetPosition(left.getCurrentPosition() + 1120);
        frontRight.setTargetPosition(right.getCurrentPosition() +1120);

        left.setPower(0.4);
        right.setPower(0.4);
        frontLeft.setPower(0.4);
        frontRight.setPower(0.4);

        while(opModeIsActive() && (left.isBusy() && right.isBusy() && frontLeft.isBusy() && frontRight.isBusy())){
            telemetry.addData("bl motor position", left.getCurrentPosition());
            telemetry.addData("br motor position", right.getCurrentPosition());
        }

        requestOpModeStop();
    }
}
