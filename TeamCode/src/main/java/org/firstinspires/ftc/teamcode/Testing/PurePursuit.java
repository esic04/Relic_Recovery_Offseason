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
    Point tgt = new Point(0, 3);
    double speed = 1.4;
    Point pos = new Point(0, 0);
    double output;
    double leftAngularSpeed, rightAngularSpeed;
    double gearRatio = 2.0/3;
    double wheelr = 2;
    double lookAhead;
    double curvature;
    double angleTgt;
    double hdgPIDout;
    PID heading = new PID(0.01, 0.000004, 0.0005, 0.2, -0.6, 0.6);


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        tgt.setPosition(0, 3);
        pos.setPosition(robot.GetPose().getX(), robot.GetPose().getY());
        profile.setInputs(0.8, 1.2, tgt, pos);

        telemetry.addData("angle", robot.sensors.getHeading());
        telemetry.update();
        stPos = robot.GetPosition();
        waitForStart();

        resetStartTime();
        while(opModeIsActive()){
            pos = robot.GetPosition();

            output = profile.getOutput(robot.GetPosition());

            if(cal.PointDistance(pos, tgt) < 0.4){
                tgt.setPosition(-3, 3);
                profile.setTarget(tgt);
            }

            curvature = (2 * robot.GetPosition().getX()) / (lookAhead * lookAhead);
            angleTgt = curvature * 90;

            hdgPIDout = heading.getOutput(robot.sensors.getHeading(), angleTgt);

            leftAngularSpeed = (output * 12 + hdgPIDout) / wheelr / gearRatio;
            rightAngularSpeed = (output * 12 - hdgPIDout) / wheelr / gearRatio;

            robot.driveTrain.left.setVelocity(leftAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.right.setVelocity(rightAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontLeft.setVelocity(leftAngularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontRight.setVelocity(rightAngularSpeed, AngleUnit.RADIANS);

        }


    }
}
