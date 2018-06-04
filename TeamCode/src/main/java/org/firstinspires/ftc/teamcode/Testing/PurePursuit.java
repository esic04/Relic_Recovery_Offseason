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
    double dist, output, desiredOut, stDist, count, stoppingAccel;
    double previousSpeed, time, preTime, deltaT, currAcel;
    double angularSpeed;
    double gearRatio = 2.0/3;
    double wheelr = 2;
    boolean decel = false, fastDecel = false, cruise, stopped;
    PID heading = new PID(0.01, 0.000004, 0.0005, 0.2, -0.6, 0.6);
    double angleTgt, headingOut;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        tgt.setPosition(0, 3);
        pos.setPosition(robot.driveTrain.CalcPose(robot.sensors.getHeading()).getX(), robot.driveTrain.CalcPose(robot.sensors.getHeading()).getY());
        profile.setInputs(0.8, 1.2, tgt, pos);

        telemetry.addData("angle", robot.sensors.getHeading());
        telemetry.update();
        stPos = robot.driveTrain.GetPosition(robot.sensors.getHeading());
        waitForStart();

        resetStartTime();
        while(opModeIsActive()){
            pos = robot.driveTrain.GetPosition(robot.sensors.getHeading());

            output = profile.getOutput(robot.driveTrain.GetPosition(robot.sensors.getHeading()));

            if(cal.PointDistance(pos, tgt) < 0.4){
                tgt.setPosition(-3, 3);
                profile.setTarget(tgt);
            }

            angularSpeed = (output * 12) / wheelr / gearRatio;

            robot.driveTrain.left.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.right.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontLeft.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontRight.setVelocity(angularSpeed, AngleUnit.RADIANS);

        }


    }
}
