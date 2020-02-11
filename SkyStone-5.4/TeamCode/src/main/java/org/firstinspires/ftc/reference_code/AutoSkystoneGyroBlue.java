package org.firstinspires.ftc.reference_code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
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

/**
 * Iggy's original autonomous code for reference purposes.
 * @author Iggy
 */

@Autonomous(name="Skystone-Gyro-Blue", group = "BlueAuton")
public class AutoSkystoneGyroBlue extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    private BNO055IMU imu;
    private DistanceSensor color;
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera phoneCam;
    private AutoSkystoneGyroBlue.DetectorPipeline detectorPipeline;
    private String stoneConfig;


    static final double     COUNTS_PER_MOTOR_REV    = 288;
    static final double     DRIVE_GEAR_REDUCTION    = 0.9 ;     // Gear ratio maybe 0.375
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     DRIVE_COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED              = 0.7;
    static final double     DRIVE_SPEED             = 1.0;
    static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.19;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.125;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {
        runtime.reset();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.init(hardwareMap);
        color = hardwareMap.get(DistanceSensor.class, "color");

        //Set up the motors and servos
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode                = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detectorPipeline = new DetectorPipeline();
        phoneCam.setPipeline(detectorPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            telemetry.addData("Status:", "Calibrating");
            sleep(50);
            idle();
        }


        while(!isStarted()){
            //Find the position of the Skystone
            stoneConfig = detectorPipeline.getDetectedPos();

            // Send telemetry message to indicate successful Encoder reset and initialization
            telemetry.addData("Status:", " Initialized");
            telemetry.addData("Detected:", stoneConfig);
            telemetry.update();
        }

        //START OF PLAY
        waitForStart();
        robot.clawT.setPosition(1);
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

        encoderDrive(DRIVE_SPEED,8, -8, -8, 8, 2.0);//Move off the wall

        if (stoneConfig.equals("Right")){
            encoderDrive(.8, 7, 7, 7, 7, 2.0);
            gyroTurn(TURN_SPEED, 90, 3.0);
            encoderDrive(DRIVE_SPEED, 20, 20, 20, 20, 4.0);

            robot.leftIn.setPower(-.9);//Run Intakes
            robot.rightIn.setPower(-.9);
            runtime.reset();
            double time;
            while(true){
                robot.leftFront.setPower(.5);//Move forward
                robot.leftRear.setPower(.5);
                robot.rightFront.setPower(.5);
                robot.rightRear.setPower(.5);
                telemetry.addData("Block:", color.getDistance(DistanceUnit.CM));
                telemetry.update();
                if(color.getDistance(DistanceUnit.CM)<20 || runtime.seconds()>3){//Checks to see if the color sensor detects a block has been picked up
                    robot.leftIn.setPower(0);//Stop all the motors
                    robot.rightIn.setPower(0);
                    robot.leftFront.setPower(0);
                    robot.leftRear.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightRear.setPower(0);
                    time = runtime.seconds();//Store how long it ran forward for
                    break;
                }
            }
            runtime.reset();
            while(runtime.seconds()<(time+.3)){//Run back how long you ran forward (plus a little bit)
                robot.leftFront.setPower(-.5);
                robot.leftRear.setPower(-.5);
                robot.rightFront.setPower(-.5);
                robot.rightRear.setPower(-.5);
            }
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);
            gyroDrive(DRIVE_SPEED, 60, 180, false, false, 4.0);

            robot.leftIn.setPower(1);//Outtake all the motors
            robot.rightIn.setPower(1);
            sleep(300);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);
            encoderDrive(DRIVE_SPEED, -71, -71, -71, -71, 8.0);

            gyroTurn(TURN_SPEED, 0, 3.0);
            //encoderDrive(.9, -6, -6, -6, -6, 3.0);
            encoderDrive(.8, 17, -17, -17, 17, 3.0);
            robot.leftIn.setPower(-.9);//Run Intakes
            robot.rightIn.setPower(-.9);
            runtime.reset();
            while (true) {
                robot.leftFront.setPower(.3);//Move forward
                robot.leftRear.setPower(.3);
                robot.rightFront.setPower(.3);
                robot.rightRear.setPower(.3);
                telemetry.addData("Block:", color.getDistance(DistanceUnit.CM));
                telemetry.update();
                if (color.getDistance(DistanceUnit.CM) < 20 || runtime.seconds() > 2.5) {//Checks to see if the color sensor detects a block has been picked up
                    robot.leftIn.setPower(0);//Stop all the motors
                    robot.rightIn.setPower(0);
                    robot.leftFront.setPower(0);
                    robot.leftRear.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightRear.setPower(0);
                    time = runtime.seconds();//Store how long it ran forward for
                    break;
                }
            }
            runtime.reset();
            robot.leftIn.setPower(-1);//Outtake all the motors
            robot.rightIn.setPower(-1);
            sleep(50);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);
            while (runtime.seconds() < (time + .2)) {//Run back how long you ran forward (plus a little bit)
                robot.leftFront.setPower(-.3);
                robot.leftRear.setPower(-.3);
                robot.rightFront.setPower(-.3);
                robot.rightRear.setPower(-.3);
            }
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);
            gyroTurn(TURN_SPEED, 90, 3.0);
            encoderDrive(.9, -13, -13, -13, -13, 2.0);

            gyroTurn(TURN_SPEED, 180, 3.0);//Rotate 90 degrees
            encoderDrive(DRIVE_SPEED, 68, 68, 68, 68, 4.0);

            robot.leftIn.setPower(1);//Outtake all the motors
            robot.rightIn.setPower(1);
            sleep(300);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);

            encoderDrive(DRIVE_SPEED, -13, -13, -13, -13, 2.0);

        }
        else if (stoneConfig.equals("Left")){
            encoderDrive(.8, -7, -7, -7, -7, 2.0);
            gyroTurn(TURN_SPEED, 90, 3.0);
            encoderDrive(DRIVE_SPEED, 20, 20, 20, 20, 4.0);

            robot.leftIn.setPower(-.9);//Run Intakes
            robot.rightIn.setPower(-.9);
            runtime.reset();
            double time;
            while(true){
                robot.leftFront.setPower(.5);//Move forward
                robot.leftRear.setPower(.5);
                robot.rightFront.setPower(.5);
                robot.rightRear.setPower(.5);
                telemetry.addData("Block:", color.getDistance(DistanceUnit.CM));
                telemetry.update();
                if(color.getDistance(DistanceUnit.CM)<20 || runtime.seconds()>3){//Checks to see if the color sensor detects a block has been picked up
                    robot.leftIn.setPower(0);//Stop all the motors
                    robot.rightIn.setPower(0);
                    robot.leftFront.setPower(0);
                    robot.leftRear.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightRear.setPower(0);
                    time = runtime.seconds();//Store how long it ran forward for
                    break;
                }
            }
            runtime.reset();
            while(runtime.seconds()<(time+.3)){//Run back how long you ran forward (plus a little bit)
                robot.leftFront.setPower(-.5);
                robot.leftRear.setPower(-.5);
                robot.rightFront.setPower(-.5);
                robot.rightRear.setPower(-.5);
            }
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);
            gyroDrive(DRIVE_SPEED, 40, 180, false, false, 4.0);

            robot.leftIn.setPower(1);//Outtake all the motors
            robot.rightIn.setPower(1);
            sleep(300);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);
            encoderDrive(DRIVE_SPEED, -62, -62, -62, -62, 8.0);

            gyroTurn(TURN_SPEED, 90, 3.0);
            robot.leftIn.setPower(-.9);//Run Intakes
            robot.rightIn.setPower(-.9);
            runtime.reset();
            while (true) {
                robot.leftFront.setPower(.5);//Move forward
                robot.leftRear.setPower(.5);
                robot.rightFront.setPower(.5);
                robot.rightRear.setPower(.5);
                telemetry.addData("Block:", color.getDistance(DistanceUnit.CM));
                telemetry.update();
                if (color.getDistance(DistanceUnit.CM) < 20 || runtime.seconds() > 3) {//Checks to see if the color sensor detects a block has been picked up
                    robot.leftIn.setPower(0);//Stop all the motors
                    robot.rightIn.setPower(0);
                    robot.leftFront.setPower(0);
                    robot.leftRear.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightRear.setPower(0);
                    time = runtime.seconds();//Store how long it ran forward for
                    break;
                }
            }
            runtime.reset();
            while (runtime.seconds() < (time-.15)) {//Run back how long you ran forward (plus a little bit)
                robot.leftFront.setPower(-.6);
                robot.leftRear.setPower(-.6);
                robot.rightFront.setPower(-.6);
                robot.rightRear.setPower(-.6);
            }
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);//Rotate 90 degrees
            encoderDrive(DRIVE_SPEED, 65, 65, 65, 65, 4.0);

            robot.leftIn.setPower(1);//Outtake all the motors
            robot.rightIn.setPower(1);
            sleep(300);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);

            encoderDrive(DRIVE_SPEED, -13, -13, -13, -13, 2.0);

        }
        else{
            gyroTurn(TURN_SPEED, 90, 3.0);
            encoderDrive(DRIVE_SPEED, 20, 20, 20, 20, 4.0);

            robot.leftIn.setPower(-.9);//Run Intakes
            robot.rightIn.setPower(-.9);
            runtime.reset();
            double time;
            while(true){
                robot.leftFront.setPower(.5);//Move forward
                robot.leftRear.setPower(.5);
                robot.rightFront.setPower(.5);
                robot.rightRear.setPower(.5);
                telemetry.addData("Block:", color.getDistance(DistanceUnit.CM));
                telemetry.update();
                if(color.getDistance(DistanceUnit.CM)<20 || runtime.seconds()>3){//Checks to see if the color sensor detects a block has been picked up
                    robot.leftIn.setPower(0);//Stop all the motors
                    robot.rightIn.setPower(0);
                    robot.leftFront.setPower(0);
                    robot.leftRear.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightRear.setPower(0);
                    time = runtime.seconds();//Store how long it ran forward for
                    break;
                }
            }
            runtime.reset();
            while(runtime.seconds()<(time+.3)){//Run back how long you ran forward (plus a little bit)
                robot.leftFront.setPower(-.5);
                robot.leftRear.setPower(-.5);
                robot.rightFront.setPower(-.5);
                robot.rightRear.setPower(-.5);
            }
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);
            gyroDrive(DRIVE_SPEED, 48, 180, false, false, 5.0);

            robot.leftIn.setPower(1);//Outtake all the motors
            robot.rightIn.setPower(1);
            sleep(300);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);
            encoderDrive(DRIVE_SPEED, -71, -71, -71, -71, 8.0);

            gyroTurn(TURN_SPEED, 90, 3.0);
            robot.leftIn.setPower(-.9);//Run Intakes
            robot.rightIn.setPower(-.9);
            runtime.reset();
            while (true) {
                robot.leftFront.setPower(.5);//Move forward
                robot.leftRear.setPower(.5);
                robot.rightFront.setPower(.5);
                robot.rightRear.setPower(.5);
                telemetry.addData("Block:", color.getDistance(DistanceUnit.CM));
                telemetry.update();
                if (color.getDistance(DistanceUnit.CM) < 20 || runtime.seconds() > 3) {//Checks to see if the color sensor detects a block has been picked up
                    robot.leftIn.setPower(0);//Stop all the motors
                    robot.rightIn.setPower(0);
                    robot.leftFront.setPower(0);
                    robot.leftRear.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightRear.setPower(0);
                    time = runtime.seconds();//Store how long it ran forward for
                    break;
                }
            }
            runtime.reset();
            while (runtime.seconds() < (time-.15)) {//Run back how long you ran forward (plus a little bit)
                robot.leftFront.setPower(-.6);
                robot.leftRear.setPower(-.6);
                robot.rightFront.setPower(-.6);
                robot.rightRear.setPower(-.6);
            }
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);

            gyroTurn(TURN_SPEED, 180, 3.0);//Rotate 90 degrees
            encoderDrive(DRIVE_SPEED, 74, 74, 74, 74, 4.0);

            robot.leftIn.setPower(1);//Outtake all the motors
            robot.rightIn.setPower(1);
            sleep(300);
            robot.leftIn.setPower(0);//Stop all the motors
            robot.rightIn.setPower(0);

            encoderDrive(DRIVE_SPEED, -13, -13, -13, -13, 2.0);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
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

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && runtime.seconds() < timeoutS) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }

        runtime.reset();

        //resetTimeCounter += runtime.seconds();
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
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
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

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else if(Math.abs(error) < 15.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.40;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 30.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.60;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 45.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.75;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 60.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.9;
            leftSpeed = -rightSpeed;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * Math.signum(steer);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftRear.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightRear.setPower(rightSpeed);

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
            newLeftTarget = robot.leftFront.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = robot.rightFront.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            //motor_left.setPower(speed * Math.signum(distance));
            //motor_right.setPower(speed * Math.signum(distance));

            // keep looping while we are still active, and BOTH motors are running.
            while   (opModeIsActive() &&
                    (Math.abs(robot.leftFront.getCurrentPosition()) < newLeftTarget ) &&
                    (Math.abs(robot.rightFront.getCurrentPosition()) < newRightTarget) &&
                    runtime.seconds() < timeoutS){

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                rawSpeed = speed;

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                double rem = ( (newLeftTarget - Math.abs(robot.leftFront.getCurrentPosition())) + (newRightTarget - Math.abs(robot.rightFront.getCurrentPosition())) ) / 2;
                double curRot = Math.abs(robot.leftFront.getCurrentPosition()) + Math.abs(robot.rightFront.getCurrentPosition()) / 2.0;
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
                
                
                robot.leftFront.setPower(leftSpeed * Math.signum(distance));
                robot.leftRear.setPower(leftSpeed * Math.signum(distance));
                robot.rightFront.setPower(rightSpeed* Math.signum(distance));
                robot.rightRear.setPower(rightSpeed* Math.signum(distance));

                // Display drive status for the driver.

                telemetry.addData("Path Status:", "In Motion");
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("rem: ", rem);
                telemetry.addData("Did Ramp Up:", rampUpOccured);
                telemetry.update();

            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);
            telemetry.addData("Path Status:", "Motion Stopped");
            telemetry.update();

            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevLeftPos = Math.abs(robot.leftFront.getCurrentPosition());
            int prevRightPos = Math.abs(robot.rightFront.getCurrentPosition());
            sleep(25);
            while( (Math.abs(robot.rightFront.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(robot.leftFront.getCurrentPosition()) - prevLeftPos != 0) ){
                prevLeftPos = Math.abs(robot.leftFront.getCurrentPosition());
                prevRightPos = Math.abs(robot.rightFront.getCurrentPosition());
                sleep(25);
                telemetry.addData("Path Status:", "Waiting for Rest");
                telemetry.update();
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(robot.rightFront.getCurrentPosition()) + Math.abs(robot.leftFront.getCurrentPosition()); //Used to keep track of encoder positions
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(robot.rightFront.getCurrentPosition()) + Math.abs(robot.leftFront.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Left:", " "+robot.leftFront.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Right:", " "+robot.rightFront.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double leftRearInches,
                             double rightFrontInches, double rightRearInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(leftFrontInches * DRIVE_COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRear.getCurrentPosition() + (int)(leftRearInches * DRIVE_COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(rightFrontInches * DRIVE_COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRear.getCurrentPosition() + (int)(rightRearInches * DRIVE_COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftRear.setTargetPosition(newLeftRearTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.leftRear.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftRear.isBusy() && robot.rightFront.isBusy() && robot.rightRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newLeftRearTarget, newRightFrontTarget, newRightRearTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.leftRear.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
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
        private final int cx0 = 75, cx1 = 140, cx2 = 220;// Width=320 Height=240
        private final int cy0 = 175, cy1 = 175, cy2 = 175;

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


            /*
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