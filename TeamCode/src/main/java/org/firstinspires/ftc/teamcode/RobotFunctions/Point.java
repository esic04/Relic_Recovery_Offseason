package org.firstinspires.ftc.teamcode.RobotFunctions;

public class Point {
    double x, y;

    public Point(double Xin, double Yin){
        x = Xin;
        y = Yin;
    }

    public void setPosition(double Xin, double Yin){
        x = Xin;
        y = Yin;
    }

    public void setX(double Xin){
        x = Xin;
    }

    public void setY(double Yin){
        y = Yin;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }
}
