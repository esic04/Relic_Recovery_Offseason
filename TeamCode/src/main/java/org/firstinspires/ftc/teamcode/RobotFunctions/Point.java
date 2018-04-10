package org.firstinspires.ftc.teamcode.RobotFunctions;

public class Point {
    double x, y;
    public Point(double inX, double inY) {
        x = inX;
        y = inY;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public void setPoint(double X, double Y){
        x = X;
        y = Y;
    }

    public void setX(double X){
        x = X;
    }

    public void setY(double Y){
        y = Y;
    }
}