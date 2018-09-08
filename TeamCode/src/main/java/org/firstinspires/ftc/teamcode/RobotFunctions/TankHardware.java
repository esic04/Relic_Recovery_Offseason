package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;

/**
 * This class interfaces subsystems (currently drivetrain and sensors) in a class for a robot using tank drive
 * To use: in linOpMode make a new tank hardware instance (TankHardware robot = new TankHardware();), then initiate it in the init period of the linOpMode.
 * To access a specific subsystem: write the name of the hardware instance, then add .subsystem name after it
 *
 * @author ethan
 * TODO: clean up this class and add commonly used variables to replace hard coded numbers
 */


public class TankHardware {
    public DriveTrain driveTrain;
    public Sensors sensors;
    LinearOpMode LinOpmode;
    OpMode opmode;
    HardwareMap hMap;
    Calculators calc = new Calculators();
    double distBetweenWheels = 16.25; //inches

    public void init(HardwareMap map, LinearOpMode LinOpMode){
        this.LinOpmode = LinOpMode;
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap, LinOpmode);
        driveTrain.left.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }

    public void init(HardwareMap map, OpMode opmode){
        this.opmode = opmode;
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap, opmode);
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