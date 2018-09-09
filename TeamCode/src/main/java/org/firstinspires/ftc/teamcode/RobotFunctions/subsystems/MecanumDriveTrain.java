package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;

/**
 * This class contains the hardware and methods used in the mecanum drivetrain
 * @see DriveTrain class for more information about drivetrain subsystems in general
 *
 * @author ethan
 */

public class MecanumDriveTrain {
    HardwareMap map;
    LinearOpMode LinOpMode; //opmodes passed to drivetrain class, allows this class to access the opmode's state
    OpMode opmode;
    public DcMotorEx left, right, frontLeft, frontRight;

    public enum motor_mode {
        run_to_position, run_with_encoder, run_without_encoder
    }

    public enum motor {
        Left, Right, FrontLeft, FrontRight
    }

    public MecanumDriveTrain(HardwareMap map, LinearOpMode LinOpMode){ //drivetrain init function for hardware class
        this.LinOpMode = LinOpMode;
        this.map = map;
        left = (DcMotorEx) map.dcMotor.get("left");
        right = (DcMotorEx) map.dcMotor.get("right");
        frontLeft = (DcMotorEx) map.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) map.dcMotor.get("frontRight");
    }

    public MecanumDriveTrain(HardwareMap map){
        this.map = map;
        left = (DcMotorEx) map.dcMotor.get("left");
        right = (DcMotorEx) map.dcMotor.get("right");
        frontLeft = (DcMotorEx) map.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) map.dcMotor.get("frontRight");
    }

    double angle, speed, rotation;
    double sinDir, cosDir;
    double power[] = new double[4];
    double largest;

    public void setMovement(double speed, double angle, double rotation){//program based on https://github.com/powerstackers/FTC-5029-Velocity-Vortex/wiki/Mecanum-Drive
        this.speed = speed;
        this.angle = angle; //angle in radians
        this.rotation = rotation;

        angle += PI / 4; // shifts angle by 45 deg because mecanum rollers are at 45

        sinDir = sin(angle);
        cosDir = cos(angle);

        power[0] = (speed * sinDir) + rotation; //front left
        power[1] = (speed * cosDir) + rotation; //front right
        power[2] = (speed * -cosDir) + rotation; //back left
        power[3] = (speed * -sinDir) + rotation; //back right

        largest = abs(power[0]); //largest power
        for (int i = 1; i < 4; i++) {
            if (abs(power[i]) > largest)
                largest = abs(power[i]); //finds largest value
        }

        // scales all powers so all powers are ≤ 1
        if (largest > 1.0) {
            for (int i = 0; i < 4; i++) {
                power[i] = power[i] / largest;
            }
        }

        left.setPower(power[2]);
        right.setPower(-power[3]);
        frontLeft.setPower(power[0]);
        frontRight.setPower(-power[1]);
    }

    double gamepadX, gamepadY, gamepadTurn;

    public void arcadeDrive(double gamepadX, double gamepadY, double gamepadTurn){//it drives (x and y values for x and y directions, gamepad turn for turning value
        this.gamepadX = gamepadX; // drive program based on https://github.com/powerstackers/FTC-5029-Velocity-Vortex/wiki/Mecanum-Drive
        this.gamepadY = gamepadY;
        this.gamepadTurn = gamepadTurn;

        rotation = gamepadTurn * 0.5; //scales movement down to be slower
        speed = Math.sqrt((gamepadY * gamepadY) + (gamepadX * gamepadX)); // use pythagorean theorem to find speed
        angle = Math.atan2(-gamepadY, gamepadX);

        setMovement(speed, angle, rotation);

    }

    double robotAngle;

    public void FieldCentricArcade(double gamepadX, double gamepadY, double gamepadTurn, double robotAngle){//it drives (x and y values for x and y directions, gamepad turn for turning value
        this.gamepadX = gamepadX; // drive program based on https://github.com/powerstackers/FTC-5029-Velocity-Vortex/wiki/Mecanum-Drive
        this.gamepadY = gamepadY;
        this.gamepadTurn = gamepadTurn;
        this.robotAngle = robotAngle;

        rotation = gamepadTurn;
        speed = Math.sqrt((gamepadY * gamepadY) + (gamepadX * gamepadX)); // use pythagorean theorem to find speed
        angle = Math.atan2(-gamepadY, gamepadX);
        robotAngle = robotAngle * (PI / 180);

        rotation *= 0.5; // scales down rotation to be more reasonable
        angle -= robotAngle;

        setMovement(speed, angle, rotation);
    }

}
