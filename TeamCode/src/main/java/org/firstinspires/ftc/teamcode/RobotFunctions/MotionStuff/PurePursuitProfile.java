package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

public class PurePursuitProfile {
    double accel, speed;
    Point tgt;

    public void setInputs(double MaxAccel, double MaxSpeed, Point target){
        accel = MaxAccel;
        speed = MaxSpeed;
        tgt = target;
    }

    public void setTarget(Point target){
        tgt = target;
    }

    double deccelDist, deccelTime;

    private double deccelDist(){
        deccelTime = speed / accel;
        deccelDist = (deccelTime * speed) / 2;
        return deccelDist;
    }
}
