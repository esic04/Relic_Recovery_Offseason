package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;

/**
 * This class interfaces multiple subsystems (currently drivetrain and sensors) together in a single class for a robot using a mecanum drive
 * There are two init methods, one for linear opmodes, and one for iterative opmodes
 * To determine if the opmode is stopped, both opmodes have to be checked if they are stopped
 *
 * @author ethan
 */

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
