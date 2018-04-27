package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by ethan on 3/16/18.
 */
@Autonomous
public class PIdTurnTest extends LinearOpMode {
    DcMotor left;
    DcMotor right;
    DcMotor frontLeft;
    DcMotor frontRight;

    BNO055IMU imu;
    Orientation angles;

    double Kp, Ki, Kd;
    double P, I, D;
    double output;
    double error;
    double preError;
    double averageError;
    double Ilimit;
    boolean stopped = false;

    @Override
    public void runOpMode(){
        left = hardwareMap.get(DcMotor.class, "left"); //old init should be replaced
        right = hardwareMap.get(DcMotor.class, "right");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Kp = 0.03f;
        Ki = 0.000001f; //tuning constants
        Kd = 0.0005f;

        Ilimit = 0.4 / Ki; //set overall I limit

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        while(opModeIsActive() && !stopped) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("angle", (angles.firstAngle + 360) % 360); //converts the -180 to 180 range of the imu to 0-360
            error = 180 - ((angles.firstAngle + 360) % 360);

            P = error;
            I = I + error;
            D = error - preError;

            if(I > Ilimit){ //limits i
                I = Ilimit;
            }

            output = (P * Kp) + (I * Ki) + (D * Kd);
            telemetry.addData("output", output);

            if(output > 1){ //limits the whole output
                output = 1;
            }

            left.setPower(output);
            frontLeft.setPower(output);
            right.setPower(output);
            frontRight.setPower(output);

            averageError = (error + preError + averageError) / 3; //was used for testing previously
            telemetry.addData("average error", averageError);
            telemetry.addData("error", error);
            preError = error;

            if((D > -0.0000001 && D < 0.0000001) && (error < 1 && error > -1)){ //checks error and rate of motor movement to break out of loop
                stopped = true;
            }
            telemetry.update();
        }

        requestOpModeStop();
    }


}
