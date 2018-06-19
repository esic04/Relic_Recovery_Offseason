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
    double lookAhead = 0.9;
    double curvature;
    double angleTgt;
    double hdgPIDout;
    PID heading = new PID(0.03, 0.000001, 0.0005, 0, 0, 0);


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        pos.setPosition(robot.GetPose().getX(), robot.GetPose().getY());
        profile.setInputs(0.8, 1.2, tgt2, pos);

        telemetry.addData("angle", robot.sensors.getHeading());
        telemetry.update();
        stPos = robot.GetPosition();
        waitForStart();

        resetStartTime();
        while(opModeIsActive()){
            pos = robot.GetPosition();

            output = profile.getOutput(robot.GetPosition());

            if(cal.PointDistance(robot.GetPosition(), tgt2) < 0.3){
                requestOpModeStop();
            }

            curvature = (2 * (profile.getTarget().getX() - robot.GetPosition().getX())) / (lookAhead * lookAhead);
            angleTgt = (curvature * 35) + robot.sensors.getHeading();

            hdgPIDout = heading.getOutput(robot.sensors.getHeading(), angleTgt);

            leftAngularSpeed = (output * 12 + hdgPIDout) / wheelr / gearRatio;
            rightAngularSpeed = (output * 12 - hdgPIDout) / wheelr / gearRatio;

            robot.driveTrain.left.setVelocity(leftAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.right.setVelocity(rightAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontLeft.setVelocity(leftAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontRight.setVelocity(rightAngularSpeed, AngleUnit.RADIANS);

            telemetry.addData("output", output);
            telemetry.addData("targetx", profile.getTarget().getX());
            telemetry.addData("left angular speed", leftAngularSpeed);
            telemetry.addData("angle tgt", angleTgt);
            telemetry.addData("robot hdg", robot.sensors.getHeading());
            telemetry.addData("robot x", robot.GetPosition().getX());
            telemetry.addData("robot y", robot.GetPosition().getY());
            telemetry.update();
        }


    }
}
