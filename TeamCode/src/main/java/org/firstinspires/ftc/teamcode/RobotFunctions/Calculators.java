package org.firstinspires.ftc.teamcode.RobotFunctions;

/**
 * This class contains methods for basic calculations such as converting encoder counts to distance in inches
 */

public class Calculators {
    double encoderDistance;
    double distanceIn;
    double distanceFt;
    double gearRatio = 2.0/3.0; //needs to have decimal otherwise int/int = 0
    double wheelDiameter = 4;
    double countsPerRevolution = 1220; //for andymark neverest 40

    public double Encoder2Inches(double encoder){
        encoderDistance = encoder;
        distanceIn = (encoderDistance / countsPerRevolution * gearRatio) * (wheelDiameter * Math.PI);
        return distanceIn;
    }

    public double Encoder2Ft(double encoder){
        encoderDistance = encoder;
        distanceFt = (encoderDistance / countsPerRevolution * gearRatio) * (wheelDiameter * Math.PI) / 12;
        return distanceFt;
    }

    Point p1, p2;
    double distance;

    public double PointDistance(Point point1, Point point2){
        p1 = point1;
        p2 = point2;

        distance = Math.sqrt((Math.pow((p2.getX() - p1.getX()), 2)) + (Math.pow((p2.getY() - p1.getY()), 2))); // distance formula
        return distance;
    }


}
