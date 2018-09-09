package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;

public class DriveDistance extends OpMode {
    TankHardware robot = new TankHardware();
    Calculators cal = new Calculators();

    @Override
    public void init(){
        robot.init(hardwareMap);
    }


    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);

        telemetry.addData("bl dist", cal.Encoder2Ft(robot.driveTrain.bl.getCurrentPosition()));
        telemetry.addData("br dist", cal.Encoder2Ft(robot.driveTrain.br.getCurrentPosition()));
    }
}
