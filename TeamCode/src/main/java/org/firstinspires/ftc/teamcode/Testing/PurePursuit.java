package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PurePursuitProfile;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

@Autonomous
public class PurePursuit extends LinearOpMode {
    Hardware robot = new Hardware();
    PurePursuitProfile profile = new PurePursuitProfile();
    Point tgt = new Point(0, 4);
    double speed;
    Point pos = new Point(0, 0);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        tgt.setPosition(0, 4);
        pos.setPosition(robot.driveTrain.CalcPose(robot.sensors.getHeading()).getX(), robot.driveTrain.CalcPose(robot.sensors.getHeading()).getY());
        profile.setInputs(0.8, 1.2, tgt, pos);

        telemetry.addData("angle", robot.sensors.getHeading());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            pos.setPosition(robot.driveTrain.CalcPose(robot.sensors.getHeading()).getX(), robot.driveTrain.CalcPose(robot.sensors.getHeading()).getY());

            speed = profile.getOutput(pos);
            robot.driveTrain.setSpeeds(speed, speed, robot.sensors.getHeading(), 90);

            telemetry.addData("angle", robot.sensors.getHeading());
            telemetry.update();

        }


    }
}
