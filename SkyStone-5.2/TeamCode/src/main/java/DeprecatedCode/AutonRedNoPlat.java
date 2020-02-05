package DeprecatedCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Stack;

@Autonomous
public class AutonRedNoPlat extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private DcMotor motor_encoder;
    private DcMotor motor_encoderCenter;
    private Servo servo_gripLeft;
    private Servo servo_gripRight;
    private Servo servo_grabber;
    private Servo servo_platformRight;
    private Servo servo_platformLeft;
    private BNO055IMU imu;
    private OpenCvCamera phoneCam;
    private DetectorPipeline detectorPipeline;
    private Stack<Double> xStack;
    private ElapsedTime runtime = new ElapsedTime();
    private String stoneConfig;
    private double resetTimeCounter = 0.0;



    static final double     COREHEX_COUNTS_PER_MOTOR_REV    = 288 ;
    static final double     HEX140_COUNTS_PER_MOTOR_REV    = 1120 ; //THIS CHANGED
    static final double     HEX120_COUNTS_PER_MOTOR_REV    = 450; //THIS CHANGED
    static final double     ENCODER_COUNTS_PER_MOTOR_REV    = 5000; //THIS CHANGED
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // Gear ratio maybe 0.375
    static final double     CENTER_GEAR_REDUCTION    = 1 ;     // Gear ratio
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    static final double     SMALL_WHEEL_DIAMETER_INCHES   = 2.3622 ;     // For figuring circumference
    static final double     DRIVE_COUNTS_PER_INCH         = (HEX120_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416); //THIS CHANGED
    static final double     CENTER_COUNTS_PER_INCH = (HEX140_COUNTS_PER_MOTOR_REV * CENTER_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416); //THIS CHANGED
    static final double     ENCODER_COUNTS_PER_INCH         = (ENCODER_COUNTS_PER_MOTOR_REV) / (SMALL_WHEEL_DIAMETER_INCHES * 3.1416);
    static final double     DRIVE_SPEED_FAST        = 0.8;
    static final double     DRIVE_SPEED_SLOW        = 0.6; //0.6
    static final double     TURN_SPEED              = 0.65;
    static final double     HEADING_THRESHOLD       = 3 ;      // As tight as we can make it with an integer gyro
    static final double     DISTANCE_THRESHOLD      = 10;      //How close the robot has to be to stop
    static final double     P_TURN_COEFF            = 0.19;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.175;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {
        runtime.reset();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detectorPipeline = new DetectorPipeline();
        phoneCam.setPipeline(detectorPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        motor_encoder = hardwareMap.get(DcMotor.class, "motor_encoder");
        motor_encoderCenter = hardwareMap.get(DcMotor.class, "motor_encoderCenter");
        servo_grabber = hardwareMap.get(Servo.class, "servo_grabber");
        servo_gripLeft = hardwareMap.get(Servo.class, "servo_gripLeft");
        servo_gripRight = hardwareMap.get(Servo.class, "servo_gripRight");
        servo_platformRight = hardwareMap.get(Servo.class, "servo_platformRight");
        servo_platformLeft = hardwareMap.get(Servo.class, "servo_platformLeft");


        motor_left.setDirection(DcMotor.Direction.REVERSE);
        motor_right.setDirection(DcMotor.Direction.FORWARD);
        motor_center.setDirection(DcMotor.Direction.FORWARD);

        //motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set up the motors and servos
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_encoderCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //POSSIBLE THAT THIS SHOULD BE DELETED
        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo_grabber.resetDeviceConfigurationForOpMode();
        servo_gripRight.resetDeviceConfigurationForOpMode();
        servo_gripLeft.resetDeviceConfigurationForOpMode();

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode                = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        while(!isStarted()){
            //Find the position of the Skystone
            stoneConfig = detectorPipeline.getDetectedPos();

            // Send telemetry message to indicate successful Encoder reset and initialization
            telemetry.addData("Status:", " Initialized");
            telemetry.addData("Encoder Center:", motor_center.getCurrentPosition());
            telemetry.addData("Detected:", stoneConfig);
            telemetry.update();
        }




        //START OF PLAY
        waitForStart();
        runtime.reset();
        telemetry.update();

        //_____________________________________________________
        //                                                     |
        //      |---------------------------------------|      |
        //      | S T A R T   O F   D R I V E   C O D E |      |
        //      |---------------------------------------|      |
        //                                                     |
        //_____________________________________________________|
        // Note: Reverse movement is obtained by setting a negative distance (not speed), negative on H-Drive is left

        //-------------
        // M I D D L E
        //-------------
        if(stoneConfig.equals("Middle")){
            //Put the arm and claws in a ready position
            raiseArm();
            unclamp();
            sleep(2000);

            //Drive forward to the block
            gyroDrive2(DRIVE_SPEED_SLOW, 35,0, true, false,10.0);

            //Lower arm onto block
            lowerArm();
            sleep(500);

            //Drive backwards with block
            gyroDrive2(0.4, -18,0, false, false,2.0);

            //Grip the block
            clamp();
            sleep(750);

            //Raise arm up
            raiseArmEvenHigher();

            //Drive up a little
            gyroDrive2(0.4, 8, 0.0, false, false, 2.0);

            //Turn to face the delivery area
            gyroTurn(TURN_SPEED, -90.0, 2.0);

            raiseArm();

            //Drive to platform
            gyroDrive2(DRIVE_SPEED_SLOW, 85,-90.0, true, true,5.0);

            //Release block onto platform
            lowerArm();
            unclamp();
            sleep(500);
            raiseArmHigher();
            sleep(250);
            clamp();
            raiseArm();

            //Drive back to block
            gyroDrive2(DRIVE_SPEED_SLOW, -124.75,-90.0, true, true,5.0);

            unclamp();
            sleep(1000);

            //Turn to face the block
            gyroTurn(TURN_SPEED, 0.0, 2.0);

            //Drive forward to the block
            gyroDrive2(DRIVE_SPEED_SLOW, 9,0, true, false,2.0);

            //Lower arm onto block
            lowerArm();
            sleep(500);

            //Drive backwards with block
            gyroDrive2(0.4, -24,0, false, false,2.0);

            //Grip the block
            clamp();
            sleep(1000);

            //Raise arm up
            raiseArmEvenHigher();

            //Drive up a little
            gyroDrive2(0.4, 9, 0.0, false, false, 2.0);

            //Turn to face the delivery area
            gyroTurn(TURN_SPEED, -90.0, 2.0);

            raiseArm();

            //Drive to delivery area
            gyroDrive2(DRIVE_SPEED_SLOW, 113,-90.0, true, true,5.0);

            //Let go of the block
            unclamp();
            sleep(1000);

            //Drive back to park on the line
            gyroDrive2(DRIVE_SPEED_SLOW, -25,-90.0, true, false,2.0);

            //Total timeouts = 34.5s
        }

        //---------
        // L E F T NOTDONE
        //---------
        else if(stoneConfig.equals("Left")){

            //Put the arm and claws in a ready position
            raiseArm();
            unclamp();

            //Drive forward to the block
            //gyroDriveDiagonal(DRIVE_SPEED_SLOW, 0.5, 30, -13.0,0, true, 3.0);
            gyroDrive2(DRIVE_SPEED_SLOW, 10,0, true, false,2.0);
            gyroDriveCenter(0.5, -3.25, 0.0, false, 1.5);
            gyroDrive2(DRIVE_SPEED_SLOW, 10,0, true, false,2.0);
            gyroDriveCenter(0.5, -3.25, 0.0, false, 1.5);
            gyroDrive2(DRIVE_SPEED_SLOW, 7,0, true, false,2.0);
            gyroDriveCenter(0.5, -3.25, 0.0, false, 1.5);

            //Lower arm onto block
            lowerArm();
            sleep(500);

            //Drive backwards with block
            gyroDrive2(DRIVE_SPEED_SLOW, -19,0, false, false,2.0);

            //Grip the block
            clamp();
            sleep(1000);

            //Raise arm up
            raiseArmHigher();

            //Drive up a little
            gyroDrive2(DRIVE_SPEED_SLOW, 7.0, 0.0, false, false, 2.0);

            //Turn to face the delivery area
            gyroTurn(TURN_SPEED, -90.0, 2.0);

            raiseArm();

            //Drive to platform
            gyroDrive2(DRIVE_SPEED_SLOW, 95,-90.0, true, false,5.0);

            //Release block onto platform
            unclamp();

            //Drive back to block
            gyroDrive2(DRIVE_SPEED_SLOW, -113.5,-90.0, true, false,5.0);

            //Turn to face the block
            sleep(1000);
            gyroTurn(TURN_SPEED, 0.0, 2.0);

            //Drive forward to the block
            gyroDriveCenter(0.5, -4, 0.0, false, 1.5);
            gyroDrive2(0.5, 16,0, false, false,2.0);


            //Lower arm onto block
            lowerLeftSideGrabber();
            sleep(500);

            //Drive backwards with block
            gyroDrive2(DRIVE_SPEED_SLOW, -18,0, false, false,2.0);
            gyroDriveCenter(0.5, 4, 0.0, false, 1.5);

            //Drive up a little
            gyroDrive2(DRIVE_SPEED_SLOW, 4.0, 0.0, false, false, 2.0);

            //Turn to face the delivery area
            gyroTurn(TURN_SPEED, -90.0, 2.0);

            //Drive to delivery area
            gyroDrive2(DRIVE_SPEED_SLOW, 110,-90.0, true, false,5.0);

            //Let go of the block
            raiseSideGrabbers();
            sleep(1000);

            //Drive back to park on the line
            gyroDrive2(DRIVE_SPEED_SLOW, -25,-90.0, true, false,2.0);

        }

        //-----------
        // R I G H T NOTDONE
        //-----------
        else if(stoneConfig.equals("Right")){

            //Put the arm and claws in a ready position
            raiseArm();
            unclamp();
            sleep(2000);

            //Drive forward to the block
            //gyroDriveDiagonal(DRIVE_SPEED_SLOW, 0.5, 32, -8.0,0, true, 3.0);

            gyroDrive2(DRIVE_SPEED_SLOW, 10,0, true, false,2.0);
            gyroDriveCenter(0.5, 3, 0.0, false, 1.5);
            gyroDrive2(DRIVE_SPEED_SLOW, 10,0, true, false,2.0);
            gyroDriveCenter(0.5, 3, 0.0, false, 1.5);
            gyroDrive2(DRIVE_SPEED_SLOW, 10,0, true, false,2.0);
            gyroDriveCenter(0.5, 2.75, 0.0, false, 1.5);

            /*
            gyroDrive2(DRIVE_SPEED_SLOW, 15,0, true, false,2.0);
            gyroTurn(TURN_SPEED, -90.0, 2.0);
            gyroDrive2(DRIVE_SPEED_SLOW, 7,0, true, false,2.0);
            gyroTurn(TURN_SPEED, 0.0, 2.0);
            gyroDrive2(DRIVE_SPEED_SLOW, 15,0, true, false,2.0);

             */

            //Lower arm onto block
            lowerArm();
            sleep(500);

            //Drive backwards with block
            gyroDrive2(DRIVE_SPEED_SLOW, -20,0, false, false,2.0);

            //Grip the block
            clamp();
            sleep(1000);

            //Raise arm up
            raiseArmEvenHigher();

            //Drive up a little
            gyroDrive2(DRIVE_SPEED_SLOW, 8, 0.0, false, false, 2.0);

            //Turn to face the delivery area
            gyroTurn(TURN_SPEED, -90.0, 2.0);

            raiseArm();

            //Drive to platform
            gyroDrive2(DRIVE_SPEED_SLOW, 72,-90.0, true, false,5.0);

            //Release block onto platform
            unclamp();

            //Drive back to block
            gyroDrive2(DRIVE_SPEED_SLOW, -105,-90.0, true, false,5.0);
            sleep(500);

            //Turn to face the block
            gyroTurn(TURN_SPEED, 0.0, 2.0);

            //Drive forward to the block
            gyroDrive2(DRIVE_SPEED_SLOW, 10,0, false, false,2.0);

            //Lower arm onto block
            lowerArm();
            sleep(500);

            //Drive backwards with block
            gyroDrive2(0.5, -28,0, false, false,2.0);

            //Grip the block
            clamp();
            sleep(1000);

            //Raise arm up
            raiseArmEvenHigher();

            //Drive up a little
            gyroDrive2(DRIVE_SPEED_SLOW, 10, 0.0, false, false, 2.0);

            //Turn to face the delivery area
            gyroTurn(TURN_SPEED, -90.0, 2.0);

            raiseArm();

            //Drive to delivery area
            gyroDrive2(DRIVE_SPEED_SLOW, 105,-90.0, true, false,5.0);

            //Let go of the block
            lowerArm();
            unclamp();
            sleep(500);
            raiseArm();

            //Drive back to park on the line
            gyroDrive2(DRIVE_SPEED_SLOW, -25,-90.0, true, false,2.0);

        }






        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void clamp(){
        servo_gripRight.setPosition(1.0);
        servo_gripLeft.setPosition(0.0);
    }

    private void unclamp(){
        servo_gripRight.setPosition(0.0);
        servo_gripLeft.setPosition(1.0);
    }

    private void raiseArm(){
        servo_grabber.setPosition(0.5);
    }

    private void raiseArmHigher(){
        servo_grabber.setPosition(0.4);
    }

    private void raiseArmEvenHigher(){
        servo_grabber.setPosition(0.3);
    }

    private void lowerArm(){
        servo_grabber.setPosition(1.0);
    }

    private void lowerSideGrabbers(){
        servo_platformRight.setPosition(0.7);
        servo_platformLeft.setPosition(0.3);
    }

    private void lowerLeftSideGrabber(){
        servo_platformLeft.setPosition(0.48);
    }

    private void lowerRightSideGrabber(){
        servo_platformRight.setPosition(0.52);
    }

    private void raiseSideGrabbers(){
        servo_platformRight.setPosition(0.0);
        servo_platformLeft.setPosition(1.0);
    }



    private void gyroDrive2(double speed,
                           double distance,
                           double angle,
                           boolean rampDown,
                           boolean rampUp,
                           double timeoutS) {

        int     newTarget;
        double  max;
        double  min;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  startPos;
        double  rawSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = (int)(motor_encoder.getCurrentPosition() + distance*ENCODER_COUNTS_PER_INCH);

            //Get current position of encoder
            startPos = motor_encoder.getCurrentPosition();

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            //motor_left.setPower(speed * Math.signum(distance));
            //motor_right.setPower(speed * Math.signum(distance));

            // keep looping while we are still active, and BOTH motors are running. Math.abs(motor_encoder.getCurrentPosition()) < newTarget
            while   (opModeIsActive() &&
                    ( (newTarget > startPos ? motor_encoder.getCurrentPosition() < newTarget : motor_encoder.getCurrentPosition() > newTarget)   ) &&
                    runtime.seconds() < timeoutS){

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                rawSpeed = speed;

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                double rem =    Math.abs(newTarget - Math.abs(motor_encoder.getCurrentPosition()));
                double curRot = Math.abs(startPos - Math.abs(motor_encoder.getCurrentPosition()));
                boolean rampUpOccured = false;

                if(curRot < 25 && rampUp){
                    rawSpeed = rawSpeed * 0.70;
                    rampUpOccured = true;
                }
                else if(curRot < 50 && rampUp){
                    rawSpeed = rawSpeed * 0.75;
                    rampUpOccured = true;
                }
                else if(curRot < 75 && rampUp){
                    rawSpeed = rawSpeed * 0.80;
                    rampUpOccured = true;
                }
                else if(curRot < 100 && rampUp){
                    rawSpeed = rawSpeed * 0.85;
                    rampUpOccured = true;
                }
                else if(curRot < 125 && rampUp){
                    rawSpeed = rawSpeed * 0.90;
                    rampUpOccured = true;
                }




                if(!rampUpOccured) {
                    if (rem < 50 && rampDown) {
                        rawSpeed = rawSpeed * 0.20;
                    } else if (rem < 100 && rampDown) {
                        rawSpeed = rawSpeed * 0.30;
                    } else if (rem < 150 && rampDown) {
                        rawSpeed = rawSpeed * 0.40;
                    } else if (rem < 300 && rampDown) {
                        rawSpeed = rawSpeed * 0.50;
                    } else if (rem < 400 && rampDown) {
                        rawSpeed = rawSpeed * 0.60;
                    } else if (rem < 500 && rampDown) {
                        rawSpeed = rawSpeed * 0.70;
                    } else if (rem < 600 && rampDown) {
                        rawSpeed = rawSpeed * 0.80;
                    } else if (rem < 700 && rampDown) {
                        rawSpeed = rawSpeed * 0.90;
                    } else if (rem < 800 && rampDown) {
                        rawSpeed= rawSpeed * 1.0;
                    }
                }



                if(rawSpeed < 0.2)
                    rawSpeed = 0.2;


                leftSpeed = rawSpeed - steer*P_DRIVE_COEFF;
                rightSpeed = rawSpeed + steer*P_DRIVE_COEFF;


                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motor_left.setPower(leftSpeed * Math.signum(distance));
                motor_right.setPower(rightSpeed* Math.signum(distance));

                // Display drive status for the driver.
                telemetry.addData("Path Status:", "In Motion");
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Is Target > Start?",  newTarget > startPos);
                telemetry.addData("Target",  "%7d:",      newTarget);
                telemetry.addData("Actual",  "%7d:",      motor_encoder.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Total Time Resetting so Far:", "%4.2f:", resetTimeCounter);
                telemetry.update();

            }

            runtime.reset();
            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            telemetry.addData("Path Status:", "Motion Stopped");
            telemetry.update();
            resetTimeCounter += runtime.seconds();
        }
    }

    private void gyroDriveDiagonal(double speed, double cSpeed,
                                   double distance, double cDistance,
                                   double angle,
                                   boolean rampDown,
                                   double timeoutS) {

        int     newTarget;
        int     newCenterTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double centerSpeed;
        double centerSteer;
        double finishCoeff;
        double rawSpeed;
        double startPos;
        double startPosCenter;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = (int)(motor_encoder.getCurrentPosition() + distance*ENCODER_COUNTS_PER_INCH);
            newCenterTarget = (int)(motor_encoderCenter.getCurrentPosition() + cDistance*ENCODER_COUNTS_PER_INCH);

            startPos = motor_encoder.getCurrentPosition();
            startPosCenter = motor_encoderCenter.getCurrentPosition();

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            cSpeed = Range.clip(Math.abs(cSpeed), 0.0, 1.0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    ( (newTarget > startPos ? motor_encoder.getCurrentPosition() < newTarget : motor_encoder.getCurrentPosition() > newTarget)
                            || (newCenterTarget > startPosCenter ? motor_encoderCenter.getCurrentPosition() < newCenterTarget : motor_encoderCenter.getCurrentPosition() > newCenterTarget) ) &&
                    runtime.seconds() < timeoutS) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                centerSteer = 1.0;
                finishCoeff = 1.0;
                rawSpeed = speed;

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                //Check to see if either center or left and right have finished, and stop them
                if((newCenterTarget > startPos ? motor_encoderCenter.getCurrentPosition() > newCenterTarget : motor_encoderCenter.getCurrentPosition() < newCenterTarget))
                    centerSteer = 0.0;
                if((newTarget > startPos ? motor_encoder.getCurrentPosition() > newTarget : motor_encoder.getCurrentPosition() < newTarget))
                    finishCoeff = 0.0;

                double rem = ( newTarget - Math.abs(motor_encoder.getCurrentPosition()) );

                if (rem < 50 && rampDown) {
                    rawSpeed = rawSpeed * 0.20;
                } else if (rem < 100 && rampDown) {
                    rawSpeed = rawSpeed * 0.30;
                } else if (rem < 150 && rampDown) {
                    rawSpeed = rawSpeed * 0.40;
                } else if (rem < 300 && rampDown) {
                    rawSpeed = rawSpeed * 0.50;
                } else if (rem < 400 && rampDown) {
                    rawSpeed = rawSpeed * 0.60;
                } else if (rem < 500 && rampDown) {
                    rawSpeed = rawSpeed * 0.70;
                } else if (rem < 600 && rampDown) {
                    rawSpeed = rawSpeed * 0.80;
                } else if (rem < 700 && rampDown) {
                    rawSpeed = rawSpeed * 0.90;
                } else if (rem < 800 && rampDown) {
                    rawSpeed= rawSpeed * 1.0;
                }

                if(rawSpeed < 0.2)
                    rawSpeed = 0.2;


                leftSpeed = (rawSpeed - steer*P_DRIVE_COEFF);
                rightSpeed = (rawSpeed + steer*P_DRIVE_COEFF);
                centerSpeed = cSpeed * centerSteer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motor_left.setPower(leftSpeed * Math.signum(distance) * finishCoeff);
                motor_right.setPower(rightSpeed * Math.signum(distance) * finishCoeff);
                motor_center.setPower(centerSpeed * Math.signum(cDistance));

                // Display drive status for the driver.
                telemetry.addData("Path Status:", "In Motion");
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:",      newTarget);
                telemetry.addData("Actual",  "%7d:",      motor_encoder.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Total Time Resetting so Far:", "%4.2f:", resetTimeCounter);
                telemetry.update();

            }

            runtime.reset();
            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            motor_center.setPower(0);
            telemetry.addData("Path Status:", "Motion Stopped");
            telemetry.update();
            resetTimeCounter += runtime.seconds();
        }
    }

    private void gyroDriveCenter(double cSpeed,
                                   double cDistance,
                                   double angle,
                                   boolean rampDown,
                                   double timeoutS) {

        int     newCenterTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double centerSpeed;
        double centerSteer;
        double finishCoeff;
        double rawSpeed;
        double startPos;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newCenterTarget = (int)(motor_encoderCenter.getCurrentPosition() + cDistance*ENCODER_COUNTS_PER_INCH);
            startPos = motor_encoderCenter.getCurrentPosition();

            runtime.reset();
            // start motion.
            cSpeed = Range.clip(Math.abs(cSpeed), 0.0, 1.0);
            //motor_left.setPower(speed * Math.signum(distance));
            //motor_right.setPower(speed * Math.signum(distance));
            //motor_center.setPower(cSpeed * Math.signum(cDistance));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (newCenterTarget > startPos ? motor_encoderCenter.getCurrentPosition() < newCenterTarget : motor_encoderCenter.getCurrentPosition() > newCenterTarget) &&
                    runtime.seconds() < timeoutS) {

                centerSpeed = cSpeed;

                motor_center.setPower(centerSpeed * Math.signum(cDistance));

                // Display drive status for the driver.
                telemetry.addData("Path Status:", "In Motion");
                telemetry.addData("Target",  "%7d:",      newCenterTarget);
                telemetry.addData("Actual",  "%7d:",      motor_center.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:",  cSpeed);
                telemetry.addData("Total Time Resetting so Far:", "%4.2f:", resetTimeCounter);
                telemetry.update();

            }

            runtime.reset();
            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            motor_center.setPower(0);
            telemetry.addData("Path Status:", "Motion Stopped");
            telemetry.update();
            resetTimeCounter += runtime.seconds();
        }
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    private void gyroTurn (  double speed, double angle, double timeoutS) {
        runtime.reset();

        int targetingCount = 0;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            // Update telemetry & Allow time for other processes to run.
            if(onHeading(speed, angle, P_TURN_COEFF)){
                if(runtime.milliseconds() > 100){
                    targetingCount++;
                    runtime.reset();
                }
            }
            else{
                targetingCount = 0;
            }

            if(targetingCount >= 5){
                break;
            }
            telemetry.update();
        }

        runtime.reset();
        resetTimeCounter += runtime.seconds();
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        motor_left.setPower(0);
        motor_right.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double targetingCount = 0;


        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else if(Math.abs(error) < 30.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.55;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 45.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.65;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 55.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.75;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 75.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.85;
            leftSpeed = -rightSpeed;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * Math.signum(steer);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motor_left.setPower(leftSpeed);
        motor_right.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    class DetectorPipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat mat0 = new Mat();
        private Mat mat1 = new Mat();
        private Mat mat2 = new Mat();

        private boolean madeMats = false;

        private Mat mask0;
        private Mat mask1;
        private Mat mask2;


        private final Scalar BLACK = new Scalar(0,0,0);
        private final Scalar WHITE = new Scalar(256, 256, 256);
        private final int r = 10;
        private final int cx0 = 100, cx1 = 150, cx2 = 230;
        private final int cy0 = 130, cy1 = 130, cy2 = 130;

        private String detectedPos;



        @Override
        public Mat processFrame(Mat input)
        {
            int h = input.height();
            int w = input.width();
            int t = input.type();
            if(!madeMats){
                mask0 = new Mat(h,w,t);
                mask1 = new Mat(h,w,t);
                mask2 = new Mat(h,w,t);
                madeMats = true;
            }

            mask0.setTo(BLACK);
            mask1.setTo(BLACK);
            mask2.setTo(BLACK);

            Imgproc.circle(mask0, new Point(cx0, cy0), r, WHITE, Core.FILLED);
            Imgproc.circle(mask1, new Point(cx1, cy1), r, WHITE, Core.FILLED);
            Imgproc.circle(mask2, new Point(cx2, cy2), r, WHITE, Core.FILLED);

            Core.bitwise_and(mask0, input, mat0);
            Core.bitwise_and(mask1, input, mat1);
            Core.bitwise_and(mask2, input, mat2);


            double sum0 = sum(Core.sumElems(mat0).val);
            double sum1 = sum(Core.sumElems(mat1).val);
            double sum2 = sum(Core.sumElems(mat2).val);

            if(sum0 < sum1 && sum0 < sum2)
                detectedPos = "Left";
            else if(sum1 < sum0 && sum1 < sum2)
                detectedPos = "Middle";
            else if(sum2 < sum1 && sum2 < sum0)
                detectedPos = "Right";


            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            Imgproc.circle(input, new Point(cx0, cy0), r, WHITE, Core.FILLED);
            Imgproc.circle(input, new Point(cx1, cy1), r, WHITE, Core.FILLED);
            Imgproc.circle(input, new Point(cx2, cy2), r, WHITE, Core.FILLED);

            return input;
        }

        public double sum(double[] arr){
            double sum = 0.0;
            for(int i = 0; i < arr.length; i++){
                sum += arr[i];
            }
            return sum;
        }

        public String getDetectedPos(){
            return  detectedPos;
        }
    }

}

//TRASH
/**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
/*
private void gyroDrive(double speed,
                       double distance,
                       double angle,
                       boolean rampDown,
                       boolean rampUp,
                       double timeoutS) {

    int     newLeftTarget;
    int     newRightTarget;
    double  max;
    double  min;
    double  error;
    double  steer;
    double  leftSpeed;
    double  rightSpeed;
    double  rawSpeed;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
        newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);

        runtime.reset();
        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        //motor_left.setPower(speed * Math.signum(distance));
        //motor_right.setPower(speed * Math.signum(distance));

        // keep looping while we are still active, and BOTH motors are running.
        while   (opModeIsActive() &&
                (Math.abs(motor_left.getCurrentPosition()) < newLeftTarget ) &&
                (Math.abs(motor_right.getCurrentPosition()) < newRightTarget) &&
                runtime.seconds() < timeoutS){

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);
            rawSpeed = speed;

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            double rem = ( (newLeftTarget - Math.abs(motor_left.getCurrentPosition())) + (newRightTarget - Math.abs(motor_right.getCurrentPosition())) ) / 2;
            double curRot = Math.abs(motor_left.getCurrentPosition()) + Math.abs(motor_right.getCurrentPosition()) / 2.0;
            boolean rampUpOccured = false;

            if(curRot < 25 && rampUp){
                rawSpeed = rawSpeed * 0.70;
                rampUpOccured = true;
            }
            else if(curRot < 50 && rampUp){
                rawSpeed = rawSpeed * 0.75;
                rampUpOccured = true;
            }
            else if(curRot < 75 && rampUp){
                rawSpeed = rawSpeed * 0.80;
                rampUpOccured = true;
            }
            else if(curRot < 100 && rampUp){
                rawSpeed = rawSpeed * 0.85;
                rampUpOccured = true;
            }
            else if(curRot < 125 && rampUp){
                rawSpeed = rawSpeed * 0.90;
                rampUpOccured = true;
            }




            if(!rampUpOccured) {
                if (rem < 50 && rampDown) {
                    rawSpeed = rawSpeed * 0.20;
                } else if (rem < 100 && rampDown) {
                    rawSpeed = rawSpeed * 0.30;
                } else if (rem < 150 && rampDown) {
                    rawSpeed = rawSpeed * 0.40;
                } else if (rem < 300 && rampDown) {
                    rawSpeed = rawSpeed * 0.50;
                } else if (rem < 400 && rampDown) {
                    rawSpeed = rawSpeed * 0.60;
                } else if (rem < 500 && rampDown) {
                    rawSpeed = rawSpeed * 0.70;
                } else if (rem < 600 && rampDown) {
                    rawSpeed = rawSpeed * 0.80;
                } else if (rem < 700 && rampDown) {
                    rawSpeed = rawSpeed * 0.90;
                } else if (rem < 800 && rampDown) {
                    rawSpeed= rawSpeed * 1.0;
                }
            }



            if(rawSpeed < 0.2)
                rawSpeed = 0.2;


            leftSpeed = rawSpeed - steer*P_DRIVE_COEFF;
            rightSpeed = rawSpeed + steer*P_DRIVE_COEFF;


            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

                /*
                min = Math.min(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if(Math.abs(leftSpeed) < Math.abs(rightSpeed) && curRot < 125) {
                    if (min < 0.4) {
                        double ratio = max / min;
                        leftSpeed = 0.4;
                        rightSpeed = 0.4 * ratio;
                    }
                }
                else if(Math.abs(leftSpeed) > Math.abs(rightSpeed) && curRot < 120){
                    if (min < 0.4) {
                        double ratio = max / min;
                        rightSpeed = 0.4;
                        leftSpeed = 0.4 * ratio;
                    }

                }
                */

/*
            motor_left.setPower(leftSpeed * Math.signum(distance));
            motor_right.setPower(rightSpeed* Math.signum(distance));

            // Display drive status for the driver.

            telemetry.addData("Path Status:", "In Motion");
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      motor_left.getCurrentPosition(),
                    motor_right.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.addData("rem: ", rem);
            telemetry.addData("Did Ramp Up:", rampUpOccured);
            telemetry.update();

        }

        // Stop all motion;
        motor_left.setPower(0);
        motor_right.setPower(0);
        telemetry.addData("Path Status:", "Motion Stopped");
        telemetry.update();

        //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
        int prevLeftPos = Math.abs(motor_left.getCurrentPosition());
        int prevRightPos = Math.abs(motor_right.getCurrentPosition());
        sleep(25);
        while( (Math.abs(motor_right.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(motor_left.getCurrentPosition()) - prevLeftPos != 0) ){
            prevLeftPos = Math.abs(motor_left.getCurrentPosition());
            prevRightPos = Math.abs(motor_right.getCurrentPosition());
            sleep(25);
            telemetry.addData("Path Status:", "Waiting for Rest");
            telemetry.update();
        }

        //Reset the encoders so they are zeroed
        double resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition()); //Used to keep track of encoder positions
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
        //Let the reset occur
        double resetTime = runtime.seconds();
        while (Math.abs(resetC) > 0){
            resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition());
            telemetry.addData("Encoder Reset Progress Left:", " "+motor_left.getCurrentPosition());
            telemetry.addData("Encoder Reset Progress Right:", " "+motor_right.getCurrentPosition()); telemetry.update();
            //Just in case the robot gets stuck in this loop, it will try to reset again every second
            if(runtime.seconds() - resetTime > 1.0){
                motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                resetTime = runtime.seconds();
            }
            //idle();
        }

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
*/


