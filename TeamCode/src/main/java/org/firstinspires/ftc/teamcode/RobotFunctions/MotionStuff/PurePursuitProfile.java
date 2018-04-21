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

    Point pos;
    double dist, output, desiredOut, stDist, count, stoppingAccel;
    double previousSpeed, time, preTime, deltaT, currAcel;
    boolean decel, fastDecel;

    public double getOutput(Point position){
        if(count == 0){
            stDist = cal.PointDistance(stPos, tgt);
            count++;
        }

        pos = position;

        dist = cal.PointDistance(pos, tgt);

        stoppingAccel = (0 - (Math.pow(previousSpeed, 2))) / (2 * dist);

        if(stoppingAccel < accel){
            desiredOut = speed;
        } else if (stoppingAccel > accel - 0.02 && stoppingAccel < accel + 0.2){
            decel = true;
        } else if (stoppingAccel > accel + 0.2){
            fastDecel = true;
        }

        time = System.currentTimeMillis() / 1000.0;
        deltaT = time - preTime;

        if(decel){
           output = -accel * deltaT + previousSpeed;
        } else if(fastDecel){
            output = stoppingAccel * deltaT + previousSpeed;
        }

        if((previousSpeed - desiredOut) / deltaT > accel){
            output = accel * deltaT + previousSpeed;
        }

        currAcel = previousSpeed - desiredOut / deltaT;

        if(currAcel > accel){
            output = accel * deltaT + previousSpeed;
        }

        previousSpeed = output;
        preTime = time;
        return output;
    }
}

