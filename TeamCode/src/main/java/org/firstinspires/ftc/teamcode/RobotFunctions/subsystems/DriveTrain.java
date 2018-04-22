package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.teamcode.RobotFunctions.Point;
import org.firstinspires.ftc.teamcode.RobotFunctions.Pose;
import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;


public class DriveTrain {
    public DcMotor left, right, frontLeft, frontRight;
    HardwareMap map;
    Calculators cal = new Calculators();
    double gamepadX, gamepadY;
    MiniPID pid1 = new MiniPID(0, 0, 0);
    MiniPID pid2 = new MiniPID(0, 0, 0);
    MiniPID pid3 = new MiniPID(0, 0, 0);
    MiniPID pid4 = new MiniPID(0, 0, 0);; //1= back left, 2 = front left, 3 = back right, 4 = front right

    double leftOut, rightOut, frontLeftOut, frontRightOut;

    public enum motor_mode {
        run_to_position, run_with_encoder, run_without_encoder
    }

    public enum motor {
        Left, Right, FrontLeft, FrontRight
    }



    public DriveTrain(HardwareMap map){ //drivetrain init function for hardware class
        this.map = map;
        left = map.dcMotor.get("left");
        right = map.dcMotor.get("right");
        frontLeft = map.dcMotor.get("frontLeft");
        frontRight = map.dcMotor.get("frontRight");
    }

    public void arcadeDrive(double gamepadX, double gamepadY){//it drives
        this.gamepadX = gamepadX;
        this.gamepadY = gamepadY;

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(gamepadY - gamepadX);
        right.setPower(gamepadY + gamepadX);
        frontLeft.setPower(gamepadY - gamepadX);
        frontRight.setPower(gamepadY + gamepadX);
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

        leftOut = pid1.getOutput(leftSpeed(), output) + leftOut; // for velocity pid, adds pid out to previous output for better control
        rightOut = pid3.getOutput(rightSpeed(), output) + rightOut;
        frontLeftOut = pid2.getOutput(frontLeftSpeed(), output) + frontLeftOut;
        frontRightOut = pid4.getOutput(frontRightSpeed(), output) + frontRightOut;

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

        leftPow = bl.getOutput(leftSpeed(), leftTgt) + leftPow; //for velocity pid, adds pid out to previous output for better control
        rightPow = br.getOutput(rightSpeed(), rightTgt) + rightPow;
        frontLeftPow = fl.getOutput(frontLeftSpeed(), leftTgt) + frontLeftPow;
        frontRightPow = fr.getOutput(frontRightSpeed(), rightTgt) + frontRightPow;

        left.setPower(leftPow);
        right.setPower(rightPow);
        frontLeft.setPower(frontLeftPow);
        frontRight.setPower(frontRightPow);

    }

    double leftSpeed, leftPreviousTime, leftCurrentTime, leftPos, leftPreviousDist, leftDist;

    public double leftSpeed(){//calculates speed based off a single encoder count
        leftPos = left.getCurrentPosition();
        leftDist = cal.Encoder2Ft(leftPos);
        leftCurrentTime = System.currentTimeMillis() / 1000.0; //converts millisec to sec, decimal needed because of int/int
        leftSpeed = -(leftDist - leftPreviousDist) / (leftCurrentTime - leftPreviousTime);
        leftPreviousTime = leftCurrentTime;
        leftPreviousDist = leftDist;
        return -leftSpeed;

    }

    double rightSpeed, rightPreviousTime, rightCurrentTime, rightPos, rightPreviousDist, rightDist;

    public double rightSpeed(){
        rightPos = right.getCurrentPosition();
        rightDist = cal.Encoder2Ft(rightPos);
        rightCurrentTime = System.currentTimeMillis() / 1000.0;
        rightSpeed = -(rightDist - rightPreviousDist) / (rightCurrentTime - rightPreviousTime);
        rightPreviousTime = rightCurrentTime;
        rightPreviousDist = rightDist;
        return -rightSpeed;
    }

    double frontLeftSpeed, frontLeftPreviousTime, frontLeftCurrentTime, frontLeftPos, frontLeftPreviousDist, frontLeftDist;

    public double frontLeftSpeed(){
        frontLeftPos = frontLeft.getCurrentPosition();
        frontLeftDist = cal.Encoder2Ft(frontLeftPos);
        frontLeftCurrentTime = System.currentTimeMillis() / 1000.0;
        frontLeftSpeed = -(frontLeftDist - frontLeftPreviousDist) / (frontLeftCurrentTime - frontLeftPreviousTime);
        frontLeftPreviousTime = frontLeftCurrentTime;
        frontLeftPreviousDist = frontLeftDist;
        return -frontLeftSpeed;

    }

    double frontRightSpeed, frontRightPreviousTime, frontRightCurrentTime, frontRightPos, frontRightPreviousDist, frontRightDist;

    public double frontRightSpeed(){
        frontRightPos = frontRight.getCurrentPosition();
        frontRightDist = cal.Encoder2Ft(frontRightPos);
        frontRightCurrentTime = System.currentTimeMillis() / 1000.0;
        frontRightSpeed = -(frontRightDist - frontRightPreviousDist) / (frontRightCurrentTime - frontRightPreviousTime);
        frontRightPreviousTime = frontRightCurrentTime;
        frontRightPreviousDist = frontRightDist;
        return -frontRightSpeed;
    }

    double Vr, Vl, Vrx, Vry = 0, omegaR;
    double hdg;
    double Vwx, Vwy;
    double Xkp, Ykp, thetaK;
    double currTime, preTime, deltaTime;
    double width = 16.25 / 12;

    Pose pos = new Pose(0, 0, 0);

    public Pose CalcPose(double Heading){// equations found from https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/
        hdg = ((Heading + 90) % 360) * (Math.PI / 180);

        currTime = System.currentTimeMillis() / 1000.0;

        Vr = (rightSpeed() + frontRightSpeed()) / 2;
        Vl = (leftSpeed() + frontLeftSpeed()) / 2;

        deltaTime = currTime - preTime;

        Vrx = (Vr + Vl) / 2;
        omegaR = (Vr - Vl) / width;

        Vwx = Vrx * Math.cos(hdg) - Vry * Math.sin(hdg);
        Vwy = Vrx * Math.sin(hdg) + Vry * Math.cos(hdg);

        Xkp = Xkp + Vwx * deltaTime;
        Ykp = Ykp + Vwy * deltaTime;

        preTime = currTime;

        pos.setPose(Xkp, Ykp, (Heading + 90) % 360);

        return pos;
    }

    public Pose GetPose(double Heading){
       return CalcPose(Heading);
    }

    Point position = new Point(0, 0);

    public Point GetPosition(double Heading){
        position.setPosition(CalcPose(Heading).getX(), CalcPose(Heading).getY());
        return position;
    }








}
