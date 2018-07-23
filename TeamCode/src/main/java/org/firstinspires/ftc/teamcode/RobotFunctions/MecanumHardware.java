package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;

public class MecanumHardware {
    public MecanumDriveTrain mecanumDrive;
    public Sensors sensors;
    HardwareMap hMap;

    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        mecanumDrive = new MecanumDriveTrain(hMap);
        mecanumDrive.left.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }
}
