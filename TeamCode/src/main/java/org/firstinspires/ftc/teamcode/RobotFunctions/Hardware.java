package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;


public class Hardware {
    public DriveTrain driveTrain;
    public Sensors sensors;
    HardwareMap hMap;
    Calculators calc = new Calculators();


    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap);
        driveTrain.left.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }
    Pose pos = new Pose(0, 0, 0);
    Point position = new Point(0, 0);
    double dist;
    double cl, cr, cfl, cfr; //current values
    double dl, dr, dfl, dfr; //delta encoder distances for motors
    double pl, pr, pfl, pfr; //previous encoder readings for all motors
    double x, y;
    double heading;
    private Pose CalcPose(){
        cl = driveTrain.left.getCurrentPosition(); cr = driveTrain.right.getCurrentPosition(); cfl = driveTrain.frontLeft.getCurrentPosition(); cfr = driveTrain.frontRight.getCurrentPosition();
        dl = cl - pl; dr = cr - pr; dfl = cfl - pfl; dfr = cfr - pfr;
        heading = sensors.getHeading();
        dist = calc.Encoder2Ft((dl + dr + dfl + dfr) / 4);

        if(heading <= 360 && heading >= 270){
            x -= Math.sin(heading * (Math.PI / 180) - 360) * dist; //converts heading to radians for cos, cos only works with radians
            y -= (Math.cos(heading * (Math.PI / 180) - 360) * dist); //subtracts certain angle to make the angle 0-90
        } else if(heading >= 180 && heading <= 270){
            x -= Math.cos(heading * (Math.PI / 180) - 270) * dist;
            y += Math.sin(heading * (Math.PI / 180) - 270) * dist;
        } else if(heading >= 90 && heading <= 180){
            x -= (Math.cos(heading * (Math.PI / 180) - 90) * dist);
            y -= Math.sin(heading * (Math.PI / 180) - 90) * dist;
        } else if(heading >= 0 && heading <= 90){
            x -= (Math.sin(heading * (Math.PI / 180)) * dist);
            y += Math.cos(heading * (Math.PI / 180)) * dist;
        }

        pl = cl; pr = cr; pfl = cfl; pfr = cfr;

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