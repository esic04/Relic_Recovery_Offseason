package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PurePursuitProfile;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

@Autonomous
public class PurePursuit extends LinearOpMode {
    Hardware robot = new Hardware();
    PurePursuitProfile profile = new PurePursuitProfile();
    double accel = 1;
    Point stPos = new Point(0, 0);
    Calculators cal = new Calculators();
    Point tgt = new Point(0, 4);
    double speed = 1.4;
    Point pos = new Point(0, 0);
    double dist, output, desiredOut, stDist, count, stoppingAccel;
    double previousSpeed, time, preTime, deltaT, currAcel;
    double angularSpeed;
    double gearRatio = 2.0/3;
    double wheelr = 2;
    boolean decel = false, fastDecel = false, cruise;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        tgt.setPosition(0, 4);
        pos.setPosition(robot.driveTrain.CalcPose(robot.sensors.getHeading()).getX(), robot.driveTrain.CalcPose(robot.sensors.getHeading()).getY());
        profile.setInputs(0.8, 1.2, tgt, pos);

        telemetry.addData("angle", robot.sensors.getHeading());
        telemetry.update();

        waitForStart();
        resetStartTime();
        while(opModeIsActive()){
            if(count == 0){
                stDist = cal.PointDistance(stPos, tgt);
                count++;
            }

            pos = robot.driveTrain.GetPosition(robot.sensors.getHeading());

            dist = cal.PointDistance(pos, tgt);

            stoppingAccel = -((0 - (Math.pow(previousSpeed, 2))) / (2 * dist));

            if(stoppingAccel < accel - 0.2){
                cruise = true;
            } else if (stoppingAccel > accel - 0.02 && stoppingAccel < accel + 0.2){
                decel = true;
            } else if (stoppingAccel > accel + 0.2){
                fastDecel = true;
            }

            time = getRuntime();
            deltaT = time - preTime;

            if(decel){
                output = -accel * deltaT + previousSpeed;
                decel = false;
            } else if(fastDecel){
                output = stoppingAccel * deltaT + previousSpeed;
                fastDecel = false;
            }

            if(cruise){
                if((Math.abs(previousSpeed - speed) / deltaT) > accel){
                    output = accel * deltaT + previousSpeed;
                } else {
                    output = speed;
                }
                cruise = false;
            }

            if(output > speed){
                output = speed;
            }

            /*

            currAcel = previousSpeed - desiredOut / deltaT;

            if(currAcel > accel){
                output = accel * deltaT + previousSpeed;
            }
            */

            angularSpeed = (output * 12) / wheelr / gearRatio;

            robot.driveTrain.left.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.right.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontLeft.setVelocity(angularSpeed, AngleUnit.RADIANS);
            robot.driveTrain.frontRight.setVelocity(angularSpeed, AngleUnit.RADIANS);

            telemetry.addData("dist", dist);
            telemetry.addData("output", output);
            telemetry.addData("stoppping acceleration", stoppingAccel);
            telemetry.addData("set desired output thingy", Math.abs((previousSpeed - desiredOut) / deltaT));
            telemetry.update();

            previousSpeed = output;
            preTime = time;

        }


    }
}
