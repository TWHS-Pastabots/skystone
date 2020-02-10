package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class TeleOpTestEncoders extends LinearOpMode {

    private DcMotor motor_fLeft;
    private DcMotor motor_fRight;
    private DcMotor motor_bLeft;
    private DcMotor motor_bRight;

    //IMU Sensor
    BNO055IMU imu;


    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        motor_fLeft = hardwareMap.get(DcMotor.class, "left_front");
        motor_fRight = hardwareMap.get(DcMotor.class, "right_front");
        motor_bLeft = hardwareMap.get(DcMotor.class, "left_rear");
        motor_bRight = hardwareMap.get(DcMotor.class, "right_rear");



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor_fLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_fRight.setDirection(DcMotor.Direction.FORWARD);
        motor_bLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_bRight.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Status", "IMU Init Complete");
        telemetry.clear();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //Provide telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("First:", ""+(-1.0*imu.getAngularOrientation().firstAngle));
            telemetry.addData("Second:", ""+(-1.0*imu.getAngularOrientation().secondAngle));
            telemetry.addData("Third:", ""+(-1.0*imu.getAngularOrientation().thirdAngle));
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", drivePower, rightPower);
            telemetry.update();
        }
    }

}
