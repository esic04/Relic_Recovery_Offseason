package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;

import java.util.Locale;

/**
 * Created by ethan on 3/16/18.
 */
@TeleOp
public class AngleTelemetry extends OpMode {
    Hardware robot = new Hardware();

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
       telemetry.addData("angle", robot.sensors.getHeading());
       telemetry.update();
    }





}
