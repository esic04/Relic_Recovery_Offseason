package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

public class PurePursuitProfile {
    double accel, speed;
    Point tgt, stPos;
    Calculators cal = new Calculators();

    public void setInputs(double MaxAccel, double MaxSpeed, Point target, Point startPos){
        accel = MaxAccel;
        speed = MaxSpeed;
        tgt = target;
        stPos = startPos;
    }

    public void setTarget(Point target){
        tgt = target;
    }

    double deccelDist, deccelTime;

    private double decelDist(){
        deccelTime = speed / accel;
        deccelDist = (deccelTime * speed) / 2;
        return deccelDist;
    }

    Point pos = new Point(0, 0);
    double dist, output, desiredOut, stDist, count, stoppingAccel;
    double previousSpeed, time, preTime, deltaT, currAcel;
    double angularSpeed;
    double gearRatio = 2.0/3;
    double wheelr = 2;
    boolean decel = false, fastDecel = false, cruise, stopped;

    public double getOutput(Point position){
        if(count == 0){
            stDist = cal.PointDistance(stPos, tgt);
            count++;
        }

        pos = position;

        dist = cal.PointDistance(pos, tgt);

        stoppingAccel = -((0 - (Math.pow(previousSpeed, 2))) / (2 * dist));

        if(stoppingAccel < accel - 0.2){
            cruise = true;
        } else if (stoppingAccel > accel - 0.02 && stoppingAccel < accel + 0.2){
            decel = true;
        } else if (stoppingAccel > accel + 0.2){
            fastDecel = true;
        }

        time = System.currentTimeMillis() / 1000.0;
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

        if(output < 0.6 && cal.PointDistance(position, tgt) < 0.1){
            stopped = true;
        }

        previousSpeed = output;
        preTime = time;

        return output;
    }
}

