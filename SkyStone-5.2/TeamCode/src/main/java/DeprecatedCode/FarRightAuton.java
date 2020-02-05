package DeprecatedCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class FarRightAuton extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private DcMotor motor_intake_left;
    private DcMotor motor_intake_right;
    private CRServo servo_arm;
    private Servo servo_grip;
    private Servo servo_hook;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime aTimer = new ElapsedTime();
    private ElapsedTime aTimer2 = new ElapsedTime();

    static final double     COREHEX_COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Core Hex Motor Encoder
    static final double     HEX140_COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // Gear ratio
    static final double     CENTER_GEAR_REDUCTION    = 1 ;     // Gear ratio
    static final double     BIG_WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     SMALL_WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     BIG_COUNTS_PER_INCH         = (COREHEX_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (BIG_WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     SMALL_COUNTS_PER_INCH         = (HEX140_COUNTS_PER_MOTOR_REV * CENTER_GEAR_REDUCTION) / (SMALL_WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {
        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        motor_intake_left = hardwareMap.get(DcMotor.class, "motor_intake_left");
        motor_intake_right = hardwareMap.get(DcMotor.class, "motor_intake_right");
        servo_arm = hardwareMap.get(CRServo.class, "servo_arm");
        servo_grip = hardwareMap.get(Servo.class, "servo_grip");
        servo_hook = hardwareMap.get(Servo.class, "servo_hook");
        

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_center.setDirection(DcMotor.Direction.FORWARD);
        motor_intake_left.setDirection(DcMotor.Direction.REVERSE);
        motor_intake_right.setDirection(DcMotor.Direction.FORWARD);

        servo_arm.resetDeviceConfigurationForOpMode();
        servo_grip.resetDeviceConfigurationForOpMode();
        servo_hook.resetDeviceConfigurationForOpMode();
        servo_grip.setPosition(0);
        servo_hook.setPosition(0);

        //Set up the motors
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting Positions:",  "L:%7d R:%7d C:&7d",
                motor_left.getCurrentPosition(),
                motor_right.getCurrentPosition());
                motor_center.getCurrentPosition();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Note: To move Right with the HDrive use positive distance
        encoderDrive(DRIVE_SPEED+.05,  29,  29, 0, 5.0, DRIVE_SPEED);  // Forward 48 Inches with 5 Sec timeout
        //servo_hook.setPosition(0.52);
        //sleep(1000);
        //encoderDrive(DRIVE_SPEED, 0, 0, 12, 5.0);     // HDrive RIGHT 12 inches w/ 5 sec timeout
        encoderDrive(TURN_SPEED,   -13, 13, 0,4.0, DRIVE_SPEED);   // Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED+.05, 10, 10, 0,4.0, DRIVE_SPEED);   // Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double rSpeed,
                             double leftInches, double rightInches, double centerInches,
                             double timeoutS, double lSpeed) {
        int newLeftTarget;
        int newRightTarget;
        int newCenterTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(leftInches * BIG_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(rightInches * BIG_COUNTS_PER_INCH);
            newCenterTarget = motor_center.getCurrentPosition() + (int)(centerInches * SMALL_COUNTS_PER_INCH);
            motor_left.setTargetPosition(newLeftTarget);
            motor_right.setTargetPosition(newRightTarget);
            motor_center.setTargetPosition(newCenterTarget);

            // Turn On RUN_TO_POSITION
            motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_center.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motor_left.setPower(Math.abs(lSpeed));
            motor_right.setPower(Math.abs(rSpeed));
            motor_center.setPower(Math.abs(rSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&(
                    ((motor_left.isBusy() && motor_right.isBusy()) || motor_center.isBusy()))) {

                // Display it for the driver.
                telemetry.addData("Target Position:",  "L:%7d R:%7d C:%7d", newLeftTarget,  newRightTarget, newCenterTarget);
                telemetry.addData("Current Position",  "L:%7d R:%7d C:%7d",
                        motor_left.getCurrentPosition(),
                        motor_right.getCurrentPosition(),
                        motor_center.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            motor_center.setPower(0);

            // Turn off RUN_TO_POSITION
            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
