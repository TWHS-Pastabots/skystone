package org.firstinspires.ftc.reference_code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */

@TeleOp(name = "Odometry System Calibration", group = "Calibration")
@Disabled
public class OdometryCalibration extends LinearOpMode {
    //Drive motors
    private DcMotor motor_fLeft;
    private DcMotor motor_fRight;
    private DcMotor motor_bLeft;
    private DcMotor motor_bRight;

    //Odometry Encoders
    private DcMotor encoder_left;
    private DcMotor encoder_right;
    private DcMotor encoder_horizontal;


    //IMU Sensor
    BNO055IMU imu;


    final double PIVOT_SPEED = 0.5;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 1141.9488791276;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        motor_fLeft = hardwareMap.get(DcMotor.class, "left_front");
        motor_fRight = hardwareMap.get(DcMotor.class, "right_front");
        motor_bLeft = hardwareMap.get(DcMotor.class, "left_rear");
        motor_bRight = hardwareMap.get(DcMotor.class, "right_rear");

        //NAME THESE
        encoder_horizontal = hardwareMap.get(DcMotor.class, "lift_motor");
        encoder_right = hardwareMap.get(DcMotor.class, "right_rear");
        encoder_left = hardwareMap.get(DcMotor.class, "left_rear");

        motor_fLeft.setDirection(DcMotor.Direction.FORWARD);
        motor_fRight.setDirection(DcMotor.Direction.REVERSE);
        motor_bLeft.setDirection(DcMotor.Direction.FORWARD);
        motor_bRight.setDirection(DcMotor.Direction.REVERSE);

        encoder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        encoder_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Initialize IMU hardware map value
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
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        while(!isStarted()) {
            telemetry.addData("Odometry System Calibration Status", "Init Complete");
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", encoder_left.getCurrentPosition());
            telemetry.addData("Vertical Right Position", encoder_right.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code\


        while(getZAngle() < 120 && opModeIsActive()){
            motor_fRight.setPower(-PIVOT_SPEED);
            motor_bRight.setPower(-PIVOT_SPEED);
            motor_fLeft.setPower(PIVOT_SPEED);
            motor_bLeft.setPower(PIVOT_SPEED);
            if(getZAngle() < 90) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", encoder_left.getCurrentPosition());
            telemetry.addData("Vertical Right Position", encoder_right.getCurrentPosition());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(encoder_left.getCurrentPosition()) + (Math.abs(encoder_right.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        //I don't understand this
        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = encoder_horizontal.getCurrentPosition()/Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", encoder_left.getCurrentPosition());
            telemetry.addData("Vertical Right Position", encoder_right.getCurrentPosition());
            telemetry.addData("Horizontal Position", encoder_horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }


    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        motor_fRight.setPower(rf);
        motor_bRight.setPower(rb);
        motor_fLeft.setPower(lf);
        motor_bLeft.setPower(lb);
    }

}
