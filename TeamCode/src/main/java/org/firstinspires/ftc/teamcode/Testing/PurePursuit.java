package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PurePursuitProfile;
import org.firstinspires.ftc.teamcode.RobotFunctions.Point;

public class PurePursuit extends LinearOpMode {
    Hardware robot = new Hardware();
    PurePursuitProfile profile = new PurePursuitProfile();
    Point tgt;

    public void runOpMode(){
        tgt.setPosition(0, 4);
        profile.setInputs();
    }
}
