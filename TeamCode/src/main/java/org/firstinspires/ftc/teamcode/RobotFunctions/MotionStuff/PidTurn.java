package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.stormbots.MiniPID;

/**
 * Created by ethan on 3/16/18.
 */

public class PidTurn {
    DcMotor left;
    DcMotor right;
    DcMotor frontLeft;
    DcMotor frontRight;

    float Kp, Ki, Kd;
    float CurrentAngle;
    float error;
    double output;
    float out;

    MiniPID pid;

    BNO055IMU imu;



        public void PidTurn(float Angle, double MaxPower, int Target, float Kp, float Ki, float Kd){
            pid.setPID(Kp, Ki, Kd);
            output = pid.getOutput(Angle, Target);
            pid.setOutputLimits(0, MaxPower);

            left.setPower(output);
            frontLeft.setPower(output);
            right.setPower(-output);
            frontRight.setPower(-output);




        }



}
