package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

/**
 * Created by ethan on 3/16/18.
 */
@TeleOp
public class AngleTelemetry extends OpMode {
    TankHardware robot = new TankHardware();

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
       telemetry.addData("angle", robot.sensors.getHeading());
       telemetry.update();
    }





}
