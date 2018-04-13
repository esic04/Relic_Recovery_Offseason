package org.firstinspires.ftc.teamcode.RobotFunctions;

/**
 * Created by ethan on 3/23/18.
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


}
