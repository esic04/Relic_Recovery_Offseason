package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;


public class Hardware {
    public DriveTrain driveTrain;
    public Sensors sensors;
    HardwareMap hMap;
    double Vr, Vl, Vrx, Vry = 0, omegaR;
    double hdg;
    double Vwx, Vwy;
    double Xkp, Ykp, thetaK;
    double currTime, preTime, deltaTime;
    double width = 16.25 / 12;
    Pose pos = new Pose(0, 0, 0);
    Point position = new Point(0, 0);

    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap);
        driveTrain.left.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }

    public Pose CalcPose(){// equations found from https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/
        hdg = ((sensors.getHeading() + 90) % 360) * (Math.PI / 180); //shift angle 90 degrees then convert into radians

        currTime = System.currentTimeMillis() / 1000.0;

        Vr = (driveTrain.rightSpeed() + driveTrain.frontRightSpeed()) / 2;
        Vl = (driveTrain.leftSpeed() + driveTrain.frontLeftSpeed()) / 2;

        deltaTime = currTime - preTime;

        Vrx = (Vr + Vl) / 2;
        omegaR = (Vr - Vl) / width;

        Vwx = Vrx * Math.cos(hdg) - Vry * Math.sin(hdg);
        Vwy = Vrx * Math.sin(hdg) + Vry * Math.cos(hdg);

        Xkp = Xkp + Vwx * deltaTime;
        Ykp = Ykp + Vwy * deltaTime;

        preTime = currTime;

        pos.setPose(Xkp, Ykp, (sensors.getHeading() + 90) % 360);

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