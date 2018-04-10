package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

/**
 * Created by ethan on 3/23/18.
 */
//input time get current area getting position
public class ProfileGenerator {
    double maxAccel; //in ft/sec^2
    double maxSpeed; //in ft/sec
    double distance; //in ft

    public void setInputs(double DesiredAccel, double DesiredMaxSpeed, double Distance){
        maxAccel = DesiredAccel;
        maxSpeed = DesiredMaxSpeed;
        distance = Distance;
    }


    double[] parameters = new double[3];
    int index;

    public double getParameters(int index){
        parameters[0] = maxAccel;
        parameters[1] = maxSpeed;
        parameters[2] = distance;
        this.index = index;
        return parameters[index];
    }


    double area, height, base;

    private double TriArea(){
        height = maxSpeed;
        base = height/maxAccel;
        area = (base * height) / 2;
        return area;
    }


    double rectArea, length;

    private double rectLength(){
        rectArea = distance - (2 * TriArea());
        length = rectArea / maxSpeed;
        return length;
    }


    double output;
    double yInt;
    double currentTime, startTime;
    int Run;

    public double getOutput(double Time){
        if(Run == 0){
            startTime = Time;
        }
        currentTime = Time - startTime;
        yInt = maxSpeed - (-maxAccel * (base + rectLength()));
        if(currentTime < base){
            output = maxAccel * currentTime;
        } else if (currentTime > base && currentTime < (rectLength() + base)){
            output = maxSpeed;
        } else if (currentTime > base + rectLength() && (currentTime < 2 * base + rectLength())){
            output = -maxAccel * currentTime + yInt;
        } else {
            output = 0;
        }
        Run++;
        return output;
    }


}
