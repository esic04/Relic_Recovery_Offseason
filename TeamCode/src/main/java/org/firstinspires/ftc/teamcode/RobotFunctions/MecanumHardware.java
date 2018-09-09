package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;

/**
 * This class interfaces multiple subsystems (currently drivetrain and sensors) together in a single class for a robot using a mecanum drive
 * @see TankHardware for more details about the hardware type of class
 *
 * @author ethan
 */

public class MecanumHardware {
    public MecanumDriveTrain mecanumDrive;
    LinearOpMode LinOpMode;
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

    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        mecanumDrive = new MecanumDriveTrain(hMap);
        mecanumDrive.left.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }


}
