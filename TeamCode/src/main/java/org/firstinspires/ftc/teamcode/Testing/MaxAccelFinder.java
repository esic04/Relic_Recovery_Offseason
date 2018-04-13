package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotFunctions.DataLogger;
import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;

/**
 * Created by ethan on 3/24/18.
 */
@Autonomous
public class MaxAccelFinder extends LinearOpMode {
    DcMotor left, right, frontLeft, frontRight;

    double averageLeft, averageRight;

    Calculators conv = new Calculators();

    DataLogger data = new DataLogger("accel data");

    public void runOpMode(){
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        data.addField("time");
        data.addField("left distance");
        data.addField("right distance");
        data.newLine();

        waitForStart();

        while(opModeIsActive() && getRuntime() < 5){
            left.setPower(1);
            right.setPower(1);
            frontRight.setPower(1);
            frontLeft.setPower(1);

            averageLeft = (left.getCurrentPosition() + frontLeft.getCurrentPosition()) / 2;
            averageRight = (right.getCurrentPosition() + frontRight.getCurrentPosition()) / 2;

            data.addField(getRuntime());
            data.addField(conv.Encoder2Ft(averageLeft));
            data.addField(conv.Encoder2Ft(averageRight));
            data.newLine();
        }
        requestOpModeStop();
    }

}
