package DeprecatedCode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Stack;

@Autonomous
@Disabled
public class AutonDragPlatformBLUE extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private Servo servo_gripLeft;
    private Servo servo_gripRight;
    private Servo servo_grabber;
    private Servo servo_platformRight;
    private Servo servo_platformLeft;
    private BNO055IMU imu;
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;
    private Stack<Double> xStack;
    private ElapsedTime runtime = new ElapsedTime();


    static final double     COREHEX_COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Core Hex Motor Encoder
    static final double     HEX140_COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // Gear ratio maybe 0.375
    static final double     CENTER_GEAR_REDUCTION    = 1 ;     // Gear ratio
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    //static final double     SMALL_WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     DRIVE_COUNTS_PER_INCH         = (COREHEX_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     CENTER_COUNTS_PER_INCH = (HEX140_COUNTS_PER_MOTOR_REV * CENTER_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.60;
    static final double     TURN_SPEED              = 0.45;
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {
        runtime.reset();

        telemetry.addData("Status", "Initializing");
        telemetry.update();


        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        servo_grabber = hardwareMap.get(Servo.class, "servo_grabber");
        servo_gripLeft = hardwareMap.get(Servo.class, "servo_gripLeft");
        servo_gripRight = hardwareMap.get(Servo.class, "servo_gripRight");
        servo_platformRight = hardwareMap.get(Servo.class, "servo_platformRight");
        servo_platformLeft = hardwareMap.get(Servo.class, "servo_platformLeft");


        motor_left.setDirection(DcMotor.Direction.REVERSE);
        motor_right.setDirection(DcMotor.Direction.FORWARD);
        motor_center.setDirection(DcMotor.Direction.FORWARD);

        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set up the motors and servos
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


        // Send telemetry message to indicate successful Encoder reset and initialization
        telemetry.addData("Status:", " Initialized");
        telemetry.update();

        //START OF PLAY
        waitForStart();
        runtime.reset();

        telemetry.update();

        //S T A R T   O F   D R I V E   C O D E
        // Note: Reverse movement is obtained by setting a negative distance (not speed), negative on H-Drive is left

        servo_grabber.setPosition(0.33);
        servo_platformRight.setPosition(0.0);
        servo_platformLeft.setPosition(1.0);
        servo_gripRight.setPosition(0.0);
        servo_gripLeft.setPosition(1.0);

        //Drive up to platform
        gyroDriveCenter(DRIVE_SPEED, 0.6,57, -5.0,0.0, false);

        //Lower grabbers
        servo_platformRight.setPosition(1.0);
        servo_platformLeft.setPosition(0.0);
        sleep(2000);

        //Drag platform back
        gyroDrive(1.0, -60, 0.0, false, 5.0);

        //Raise grabbers
        servo_platformRight.setPosition(0.0);
        servo_platformLeft.setPosition(1.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
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
                           double timeoutS) {

        int     newLeftTarget;
        int     newRightTarget;
        double  max;
        double  min;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motor_left.setPower(speed * Math.signum(distance));
            motor_right.setPower(speed * Math.signum(distance));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (Math.abs(motor_left.getCurrentPosition()) < newLeftTarget ) &&
                    (Math.abs(motor_right.getCurrentPosition()) < newRightTarget) &&
                    runtime.seconds() < timeoutS) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                double rem = ( (newLeftTarget - Math.abs(motor_left.getCurrentPosition())) + (newRightTarget - Math.abs(motor_right.getCurrentPosition())) ) / 2;
                double curRot = Math.abs(motor_left.getCurrentPosition()) + Math.abs(motor_right.getCurrentPosition()) / 2.0;

                /*
                if(curRot < 25 && rampDown){
                    speed = speed * 0.70;
                }
                else if(curRot < 50 && rampDown){
                    speed = speed * 0.75;
                }
                else if(curRot < 75 && rampDown){
                    speed = speed * 0.80;
                }
                else if(curRot < 100 && rampDown){
                    speed = speed * 0.85;
                }
                else if(curRot < 125 && rampDown){
                    speed = speed * 0.90;
                }
                */



                if(rem < 50 && rampDown){
                    speed = speed*0.35;
                }
                else if(rem < 100 && rampDown){
                    speed = speed*0.40;
                }

                else if(rem < 150 && rampDown){
                    speed = speed*0.60;
                }

                else if(rem < 200 && rampDown){
                    speed = speed*0.80;
                }
                /*
                else if(rem < 250 && rampDown){
                    speed = speed*0.70;
                }
                else if(rem < 300 && rampDown){
                    speed = speed*0.80;
                }
                else if(rem < 350 && rampDown){
                    speed = speed*0.90;
                }

                 */

                if(speed < 0.2)
                    speed = 0.2;


                leftSpeed = speed - steer*P_DRIVE_COEFF;
                rightSpeed = speed + steer*P_DRIVE_COEFF;

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


                motor_left.setPower(leftSpeed * Math.signum(distance));
                motor_right.setPower(rightSpeed* Math.signum(distance));

                // Display drive status for the driver.

                telemetry.addData("Path Status:", "In Motion");
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      motor_left.getCurrentPosition(),
                        motor_right.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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

    private void gyroDriveCenter(double speed, double cSpeed,
                           double distance, double cDistance,
                           double angle,
                           boolean rampDown) {

        int     newLeftTarget;
        int     newRightTarget;
        int     newCenterTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double centerSpeed;
        double centerSteer;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
            newCenterTarget = motor_center.getCurrentPosition() + (int)(Math.abs(cDistance) * CENTER_COUNTS_PER_INCH);

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            cSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motor_left.setPower(speed * Math.signum(distance));
            motor_right.setPower(speed * Math.signum(distance));
            motor_center.setPower(cSpeed * Math.signum(cDistance));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    ((Math.abs(motor_left.getCurrentPosition()) < newLeftTarget ) && (Math.abs(motor_right.getCurrentPosition()) < newRightTarget))
                    || Math.abs(motor_center.getCurrentPosition()) < newCenterTarget) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                centerSteer = 1.0;

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                if(Math.abs(motor_center.getCurrentPosition()) > newCenterTarget)
                    centerSteer = 0.0;

                double rem = ( (newLeftTarget - Math.abs(motor_left.getCurrentPosition())) + (newRightTarget - Math.abs(motor_right.getCurrentPosition())) ) / 2;

                if(rem < 50 && rampDown){
                    speed = speed*0.35;
                }
                else if(rem < 100 && rampDown){
                    speed = speed*0.40;
                }

                else if(rem < 150 && rampDown){
                    speed = speed*0.60;
                }

                else if(rem < 200 && rampDown){
                    speed = speed*0.80;
                }
                /*
                else if(rem < 150 && rampDown){
                    speed = speed*0.50;
                }
                else if(rem < 200 && rampDown){
                    speed = speed*0.60;
                }
                else if(rem < 250 && rampDown){
                    speed = speed*0.70;
                }
                else if(rem < 300 && rampDown){
                    speed = speed*0.80;
                }
                else if(rem < 350 && rampDown){
                    speed = speed*0.90;
                }

                 */

                if(speed < 0.2)
                    speed = 0.2;


                leftSpeed = speed - steer*P_DRIVE_COEFF;
                rightSpeed = speed + steer*P_DRIVE_COEFF;
                centerSpeed = cSpeed * centerSteer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motor_left.setPower(leftSpeed * Math.signum(distance));
                motor_right.setPower(rightSpeed* Math.signum(distance));
                motor_center.setPower(centerSpeed * Math.signum(cDistance));

                // Display drive status for the driver.

                telemetry.addData("Path Status:", "In Motion");
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      motor_left.getCurrentPosition(),
                        motor_right.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();

            }

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            motor_center.setPower(0);
            telemetry.addData("Path Status:", "Motion Stopped");
            telemetry.update();

            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevLeftPos = Math.abs(motor_left.getCurrentPosition());
            int prevRightPos = Math.abs(motor_right.getCurrentPosition());
            int prevCenterPos = Math.abs(motor_center.getCurrentPosition());
            sleep(25);
            while( (Math.abs(motor_right.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(motor_left.getCurrentPosition()) - prevLeftPos != 0) && (Math.abs(motor_center.getCurrentPosition()) - prevCenterPos != 0)){
                prevLeftPos = Math.abs(motor_left.getCurrentPosition());
                prevRightPos = Math.abs(motor_right.getCurrentPosition());
                prevCenterPos = Math.abs(motor_center.getCurrentPosition());
                sleep(25);
                telemetry.addData("Path Status:", "Waiting for Rest");
                telemetry.update();
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition()) + Math.abs(motor_center.getCurrentPosition()); //Used to keep track of encoder positions
            motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition()) + Math.abs(motor_center.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Left:", " "+motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Right:", " "+motor_right.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && runtime.seconds() < timeoutS) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
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

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else if(Math.abs(error) < 10.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.33;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 20.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.7;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 30.0){
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

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double rSpeed, double lSpeed,
                             double rightInches, double leftInches,
                             double timeoutS, boolean rampDown){
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(leftInches) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(rightInches) * DRIVE_COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            motor_left.setPower(lSpeed * (leftInches/Math.abs(leftInches)));
            motor_right.setPower(rSpeed * (rightInches/Math.abs(rightInches)));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.abs(motor_left.getCurrentPosition()) < newLeftTarget ) &&
                    (Math.abs(motor_right.getCurrentPosition()) < newRightTarget)){

                double nlSpeed;
                double nrSpeed;
                double rem = ( (newLeftTarget - Math.abs(motor_left.getCurrentPosition())) + (newRightTarget - Math.abs(motor_right.getCurrentPosition())) ) / 2;

                if(rem < 50 && rampDown){
                    nlSpeed = rSpeed*0.5;
                    nrSpeed = lSpeed*0.5;
                }
                else {
                    nlSpeed = lSpeed;
                    nrSpeed = rSpeed;
                }
                motor_left.setPower(nlSpeed * (leftInches/Math.abs(leftInches)));
                motor_right.setPower(nrSpeed * (rightInches/Math.abs(rightInches)));

                // Display info about encoders and robot's position/heading for the driver.
                telemetry.addData("Target Position:",  "L:%7d R:%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Current Position",  "L:%7d R:%7d", motor_left.getCurrentPosition(), motor_right.getCurrentPosition());
                telemetry.addData("Remaining:", ""+rem);
            }
            telemetry.addData("Path Status:", "Finished driving"); telemetry.update();

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            telemetry.addData("Path Status:", "Motion stopped"); telemetry.update();


            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevLeftPos = Math.abs(motor_left.getCurrentPosition());
            int prevRightPos = Math.abs(motor_right.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_right.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(motor_left.getCurrentPosition()) - prevLeftPos != 0) ){
                prevLeftPos = motor_left.getCurrentPosition();
                prevRightPos = motor_right.getCurrentPosition();
                sleep(100);
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
        }
    }
    public void encoderDriveCenter(double speed, double inches, double timeoutS, boolean rampDown){
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = motor_center.getCurrentPosition() + (int)(Math.abs(inches) * CENTER_COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            motor_center.setPower(speed * Math.signum(inches));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.abs(motor_center.getCurrentPosition()) < newTarget )){

                double nSpeed;
                double rem = (newTarget - Math.abs(motor_center.getCurrentPosition()));

                if(rem < 50 && rampDown){
                    nSpeed = speed*0.5;
                }
                else {
                    nSpeed = speed;
                }
                motor_center.setPower(nSpeed * Math.signum(inches));

                // Display info about encoders and robot's position/heading for the driver.
                telemetry.addData("Target Position:",  "C:%7d", newTarget);
                telemetry.addData("Current Position",  "C:%7d", motor_center.getCurrentPosition());
                telemetry.addData("Remaining:", ""+rem);
                telemetry.update();
            }
            telemetry.addData("Path Status:", "Finished driving"); telemetry.update();

            // Stop all motion;
            motor_center.setPower(0);
            telemetry.addData("Path Status:", "Motion stopped"); telemetry.update();


            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevPos = Math.abs(motor_center.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_center.getCurrentPosition()) - prevPos != 0) ){
                prevPos = Math.abs(motor_center.getCurrentPosition());
                sleep(100);
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_center.getCurrentPosition()); //Used to keep track of encoder positions
            motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_center.getCurrentPosition()) ;
                telemetry.addData("Encoder Reset Progress Center:", " "+motor_center.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}


