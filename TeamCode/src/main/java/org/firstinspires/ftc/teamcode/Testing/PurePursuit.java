package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PurePursuitProfile;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

@Autonomous
public class PurePursuit extends LinearOpMode {
    Hardware robot = new Hardware();
    PurePursuitProfile profile = new PurePursuitProfile();
    double accel = 1;
    Point stPos = new Point(0, 0);
    Calculators cal = new Calculators();
    Point tgt = new Point(0, 2);
    Point tgt2 = new Point(-3, 3);
    double speed = 1.4;
    Point pos = new Point(0, 0);
    double output;
    double leftAngularSpeed, rightAngularSpeed;
    double gearRatio = 2.0/3;
    double wheelr = 2;
    double lookAhead = 0.5;
    double curvature;
    double[] line1 = new double[2]; //slope, then y int
    double[] line2 = new double[2]; //direct lines of path
    double[] line3 = new double[2]; //perpendicular line from robot to path line
    Point path = new Point(0, 0); //point on path closest to robot
    Point robotTgt = new Point(0, 0);
    double x, y; //x and y values of the path point
    double dist; //dist from path point to robot
    double angleTgt;
    double lineAngle;
    double hdgPIDout;
    PID heading = new PID(0.01, 0.000001, 0.0005, 0, 0, 0);


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        pos.setPosition(robot.GetPose().getX(), robot.GetPose().getY());
        profile.setInputs(0.8, 1.2, tgt2, pos);

        telemetry.addData("angle", robot.sensors.getHeading());
        telemetry.update();
        stPos = robot.GetPosition();
        robotTgt = tgt;

        line1[0] = (tgt.getY() - stPos.getY()) / (tgt.getX() - stPos.getX());
        line1[1] = tgt.getY() - (line1[0] * tgt.getX());
        line2[0] = (tgt2.getY() - tgt.getY()) / (tgt2.getX() - tgt.getX());
        line2[1] = tgt2.getY() - (line2[0] * tgt2.getX());

        waitForStart();

        resetStartTime();
        while(opModeIsActive()){
            pos = robot.GetPosition();

            output = profile.getOutput(robot.GetPosition());

            if(cal.PointDistance(robot.GetPosition(), tgt2) < 0.3){
                requestOpModeStop();
            }

            if(cal.PointDistance(robot.GetPosition(), tgt) < 0.3){
                robotTgt = tgt2;
            }

            if(robotTgt == tgt){
                if(Double.isNaN(line1[0])){ //checks if line is vertical, if so set perpendicular line's slope to 0
                    line3[0] = 0;
                }else {
                    line3[0] = 1 / line1[0];
                }
                line3[1] = pos.getY() - (line3[0] * pos.getX());
            } else if(robotTgt == tgt2){
                if(Double.isNaN(line2[0])){
                    line3[0] = 0;
                }else {
                    line3[0] = 1 / line2[0];
                }
                line3[1] = pos.getY() - (line3[0] * pos.getX());
            }

            if(robotTgt == tgt){
                x = (line1[1] - line3[1]) / (line1[0] - line3[0]);
                y = (line1[0] * x) + line1[1];
            } else if(robotTgt == tgt2){
                x = (line2[1] - line3[1]) / (line2[0] - line3[0]);
                y = (line2[0] * x) + line2[1];
            }

            path.setPosition(x, y);
            dist = cal.PointDistance(pos, path);

            if(Double.isNaN(dist)){
                dist = 0;
            }

            if(robotTgt == tgt){
                if(Double.isNaN(line1[0])){
                    lineAngle = 0;
                }else {
                    lineAngle = ((-1 * Math.atan2(tgt.getX() - stPos.getX(), tgt.getY() - stPos.getY())) * (180 / Math.PI) + 90) % 360;
                }
            } else if(robotTgt == tgt2){
                if(Double.isNaN(line2[0])){
                    lineAngle = 0;
                }else {
                    lineAngle = ((-1 * Math.atan2(tgt2.getX() - tgt.getX(), tgt2.getY() - tgt.getY())) * (180 / Math.PI) + 90) % 360;
                }
            }

            curvature = 2 * (dist) / (lookAhead * lookAhead);
            angleTgt = (curvature * 35) + lineAngle;

            hdgPIDout = heading.getOutput(robot.sensors.getHeading(), angleTgt);

            leftAngularSpeed = (output * 12 - hdgPIDout) / wheelr / gearRatio;
            rightAngularSpeed = (output * 12 + hdgPIDout) / wheelr / gearRatio;

            robot.driveTrain.left.setVelocity(leftAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.right.setVelocity(rightAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontLeft.setVelocity(leftAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontRight.setVelocity(rightAngularSpeed, AngleUnit.RADIANS);

            telemetry.addData("output", output);
            telemetry.addData("targetx", profile.getTarget().getX());
            telemetry.addData("left angular speed", leftAngularSpeed);
            telemetry.addData("angle tgt", angleTgt);
            telemetry.addData("robot hdg", robot.sensors.getHeading());
            telemetry.addData("path x", x);
            telemetry.addData("path y", y);
            telemetry.update();
        }


    }
}
