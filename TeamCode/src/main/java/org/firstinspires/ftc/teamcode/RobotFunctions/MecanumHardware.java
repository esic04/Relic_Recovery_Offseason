package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;

public class MecanumHardware {
    public MecanumDriveTrain mecanumDrive;
    LinearOpMode LinOpMode;
    OpMode opmode;
    public Sensors sensors;
    HardwareMap hMap;

    public void init(HardwareMap map, LinearOpMode LinOpMode){
        this.LinOpMode = LinOpMode;
        hMap = map; //stores hardware map
        mecanumDrive = new MecanumDriveTrain(hMap, LinOpMode);
        mecanumDrive.left.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }

    public void init(HardwareMap map, OpMode opmode){
        this.opmode = opmode;
        hMap = map; //stores hardware map
        mecanumDrive = new MecanumDriveTrain(hMap, opmode);
        mecanumDrive.left.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }


}
