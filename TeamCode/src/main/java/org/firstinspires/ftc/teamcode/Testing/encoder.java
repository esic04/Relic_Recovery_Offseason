package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.calculators;

/**
 * Created by ethan on 3/6/18.
 */
@Autonomous
public class encoder extends OpMode {
    DcMotor left;
    DcMotor right;
    DcMotor frontLeft;
    DcMotor frontRight;

    int encoderLeftDist;
    int encoderRightDist;
    double leftDist;
    double rightDist;

    Hardware h = new Hardware();

    calculators convert = new calculators();
    @Override
    public void init(){
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        left.setPower(0.4);
        right.setPower(-0.4);
        frontRight.setPower(-0.4);
        frontLeft.setPower(0.4);

        telemetry.addData("left Pos", left.getCurrentPosition());
        telemetry.addData("right pos", right.getCurrentPosition());

        encoderLeftDist = (left.getCurrentPosition() + frontLeft.getCurrentPosition()) / 2;
        encoderRightDist = (right.getCurrentPosition() + frontRight.getCurrentPosition()) / 2;

        leftDist = convert.Encoder2Inches(encoderLeftDist);
        rightDist = convert.Encoder2Inches(encoderRightDist);

        telemetry.addData("left distance", "% ft", leftDist );
        telemetry.addData("right distance", "% ft", rightDist );
    }
}
