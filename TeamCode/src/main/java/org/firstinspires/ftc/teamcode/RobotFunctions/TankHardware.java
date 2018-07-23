package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;
import org.firstinspires.ftc.teamcode.RobotFunctions.Pose;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;


public class TankHardware {
    public DriveTrain driveTrain;
    public Sensors sensors;
    HardwareMap hMap;
    Calculators calc = new Calculators();
    double distBetweenWheels = 16.25; //inches


    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap);
        driveTrain.left.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }
    Pose pos = new Pose(0, 0, 0);
    Point position = new Point(0, 0);
    double dist, leftDist, rightDist;
    double cl, cr, cfl, cfr; //current values
    double dl, dr, dfl, dfr; //delta encoder distances for motors
    double pl, pr, pfl, pfr; //previous encoder readings for all motors
    double x, y, prevX, prevY;
    double heading, prevHeading;
    private Pose CalcPose(){
        cl = driveTrain.left.getCurrentPosition(); cr = driveTrain.right.getCurrentPosition(); cfl = driveTrain.frontLeft.getCurrentPosition(); cfr = driveTrain.frontRight.getCurrentPosition();
        dl = cl - pl; dr = cr - pr; dfl = cfl - pfl; dfr = cfr - pfr;

        dist = calc.Encoder2Ft((dl + dr + dfl + dfr) / 4);
        leftDist = calc.Encoder2Ft((dl + dfl) / 2);
        rightDist = calc.Encoder2Ft((dr + dfr) / 2);

        heading = prevHeading + ((rightDist - leftDist) / (distBetweenWheels / 12));
        x = prevX - dist * Math.sin(prevHeading);
        y = prevY + dist * Math.cos(prevHeading);

        pl = cl; pr = cr; pfl = cfl; pfr = cfr;

        prevX = x; prevY = y; prevHeading = heading;

        pos.setPose(x, y, heading);

        return pos;
    }

    public Pose GetPose(){
        return CalcPose();
    }

    public Point GetPosition(){
        position.setPosition(CalcPose().getX(), CalcPose().getY());
        return position;
    }

}