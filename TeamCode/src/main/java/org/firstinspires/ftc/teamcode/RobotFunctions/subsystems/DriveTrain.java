package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;

/**
 * This class adds hardware and methods for a tank drivetrain
 * The constructor needs a linear opmode, because any loops in this class are stopped when the opmode is stopped
 * Motors are declared as DcMotorEx because the DcMotorEx provides more useful functions
 *
 * TODO: clean up and add methods, switch from mini pid to custom pid
 * @author ethan
 */


public class DriveTrain {
    //public DcMotorEx left, right, frontLeft, frontRight;
    HardwareMap map;
    LinearOpMode linOpmode;
    Calculators cal = new Calculators();

    public DcMotorEx left, right, frontLeft, frontRight;

    double gamepadX, gamepadY;
    MiniPID pid1 = new MiniPID(0, 0, 0);
    MiniPID pid2 = new MiniPID(0, 0, 0);
    MiniPID pid3 = new MiniPID(0, 0, 0);
    MiniPID pid4 = new MiniPID(0, 0, 0); //1= back left, 2 = front left, 3 = back right, 4 = front right

    double leftOut, rightOut, frontLeftOut, frontRightOut;

    public enum motor_mode {
        run_to_position, run_with_encoder, run_without_encoder
    }

    public enum motor {
        left, right, frontLeft, frontRight
    }

    public DriveTrain(HardwareMap map, LinearOpMode linOpmode){ //drivetrain init function for hardware class
        this.map = map;
        this.linOpmode = linOpmode;
        left = (DcMotorEx) map.dcMotor.get("left");
        right = (DcMotorEx) map.dcMotor.get("right");
        frontLeft = (DcMotorEx) map.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) map.dcMotor.get("frontRight");
    }

    public DriveTrain(HardwareMap map){ //drivetrain init function for hardware class
        this.map = map;
        this.linOpmode = linOpmode;
        left = (DcMotorEx) map.dcMotor.get("left");
        right = (DcMotorEx) map.dcMotor.get("right");
        frontLeft = (DcMotorEx) map.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) map.dcMotor.get("frontRight");
    }

    public void arcadeDrive(double gamepadX, double gamepadY){//it drives
        this.gamepadX = gamepadX;
        this.gamepadY = gamepadY;

        left.setPower(-gamepadY + gamepadX);
        right.setPower(-gamepadY - gamepadX);
        frontLeft.setPower(-gamepadY + gamepadX);
        frontRight.setPower(-gamepadY - gamepadX);
    }

    public void resetEncoders(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void SetBrake(){
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setMode(motor_mode mode){
        if(mode == motor_mode.run_without_encoder){
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else if(mode == motor_mode.run_with_encoder){
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else if(mode == motor_mode.run_to_position){
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    double output;
    double Kp;
    double Ki;
    double Kd;

    public void runProfile(double output){
        this.output = output;

        Kp = 0.03;
        Ki = 0.000007;
        Kd = 0;

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pid1 = new MiniPID(Kp, Ki, Kd);
        pid2 = new MiniPID(Kp, Ki, Kd);
        pid3 = new MiniPID(Kp, Ki, Kd);
        pid4 = new MiniPID(Kp, Ki, Kd);

        pid1.setDirection(true);
        pid2.setDirection(true);

        leftOut = pid1.getOutput(speed(motor.left), output) + leftOut; // for velocity pid, adds pid out to previous output for better control
        rightOut = pid3.getOutput(speed(motor.right), output) + rightOut;
        frontLeftOut = pid2.getOutput(speed(motor.frontLeft), output) + frontLeftOut;
        frontRightOut = pid4.getOutput(speed(motor.frontRight), output) + frontRightOut;

        left.setPower(leftOut);
        right.setPower(rightOut);
        frontLeft.setPower(frontLeftOut);
        frontRight.setPower(frontRightOut);

    }

    MiniPID bl, br, fl, fr, angle;
    double leftPow, rightPow, frontLeftPow, frontRightPow;
    double leftTgt, rightTgt, headingOut;

    public void setSpeeds(double leftSpeed, double rightSpeed, double heading, double headingTgt){
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        bl = new MiniPID(Kp, Ki, Kd);
        br = new MiniPID(Kp, Ki, Kd);
        fl = new MiniPID(Kp, Ki, Kd);
        fr = new MiniPID(Kp, Ki, Kd);
        angle = new MiniPID(0.03, 0.000007, 0);

        headingOut = angle.getOutput((heading + 90) % 360, headingTgt);

        leftTgt = leftSpeed - headingOut;
        rightTgt = rightSpeed + headingOut;

        br.setDirection(true);
        fr.setDirection(true);

        leftPow = bl.getOutput(speed(motor.left), leftTgt) + leftPow; //for velocity pid, adds pid out to previous output for better control
        rightPow = br.getOutput(speed(motor.right), rightTgt) + rightPow;
        frontLeftPow = fl.getOutput(speed(motor.frontLeft), leftTgt) + frontLeftPow;
        frontRightPow = fr.getOutput(speed(motor.frontRight), rightTgt) + frontRightPow;

        left.setPower(leftPow);
        right.setPower(rightPow);
        frontLeft.setPower(frontLeftPow);
        frontRight.setPower(frontRightPow);
    }

    double prevTime, time, deltaTime;
    double leftPos, leftPreviousDist, leftDist;
    double rightPos, rightPreviousDist, rightDist;
    double frontLeftPos, frontLeftPreviousDist, frontLeftDist;
    double frontRightPos, frontRightPreviousDist, frontRightDist;
    double speed;
    public double speed(motor motor){//calculates linear speed of each wheel (ft/sec)
        time = System.currentTimeMillis() / 1000.0; //decimal needed because of int/int
        deltaTime = time - prevTime;
        if(motor == DriveTrain.motor.left){
            leftPos = left.getCurrentPosition();
            leftDist = cal.Encoder2Ft(leftPos);
            speed = -(leftDist - leftPreviousDist) / (deltaTime);
            leftPreviousDist = leftDist;
        } else if(motor == DriveTrain.motor.right){
            rightPos = right.getCurrentPosition();
            rightDist = cal.Encoder2Ft(rightPos);
            speed = -(rightDist - rightPreviousDist) / (deltaTime);
            rightPreviousDist = rightDist;
        } else if(motor == DriveTrain.motor.frontLeft){
            frontLeftPos = frontLeft.getCurrentPosition();
            frontLeftDist = cal.Encoder2Ft(frontLeftPos);
            speed = -(frontLeftDist - frontLeftPreviousDist) / (deltaTime);
            frontLeftPreviousDist = frontLeftDist;
        } else if(motor == DriveTrain.motor.frontRight){
            frontRightPos = frontRight.getCurrentPosition();
            frontRightDist = cal.Encoder2Ft(frontRightPos);
            speed = -(frontRightDist - frontRightPreviousDist) / (deltaTime);
            frontRightPreviousDist = frontRightDist;
        }
        prevTime = time;

        return speed;
    }












}
