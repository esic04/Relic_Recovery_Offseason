package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;


public class Hardware {
    HardwareMap hMap;
    public DriveTrain driveTrain;
    public Sensors sensors;

    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap);
        driveTrain.left.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }
}