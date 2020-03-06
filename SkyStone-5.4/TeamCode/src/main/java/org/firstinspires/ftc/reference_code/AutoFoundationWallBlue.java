package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Iggy's original autonomous code for reference purposes.
 */

@Autonomous(name="Blue-Foundation-Wall", group="BlueAuton")
@Disabled
public class AutoFoundationWallBlue extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .9 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 1.0;
    
    private DistanceSensor distance;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distance;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftFront.getCurrentPosition(),
                          robot.leftRear.getCurrentPosition(),
                          robot.rightFront.getCurrentPosition(),
                          robot.rightRear.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        robot.leftH.setPosition(1);//unhook
        robot.rightH.setPosition(0);
        encoderDrive(DRIVE_SPEED, -22, 22, 22, -22, 4.0);//drive to foundation
        encoderDrive(DRIVE_SPEED, 14, 14, 14, 14, 3.0);
        encoderDrive(DRIVE_SPEED, -18, 18, 18, -18, 4.0);
        robot.leftH.setPosition(0); //hook foundation
        robot.rightH.setPosition(1);
        sleep(1100);
        runtime.reset();
        while(true){
            robot.leftFront.setPower(.9);
            robot.leftRear.setPower(-.9);
            robot.rightFront.setPower(-.9);
            robot.rightRear.setPower(.9);
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
            if(distance.getDistance(DistanceUnit.CM)<6 || runtime.seconds()>5){
                break;
            }
        }
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
        sleep(100);
        robot.leftH.setPosition(1);//unhook
        robot.rightH.setPosition(0);
        sleep(1500);
        encoderDrive(DRIVE_SPEED, -20, -20, -20, -20, 5.0);
        encoderDrive(DRIVE_SPEED, -3, 3, 3, -3, 1.0);
        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5.0);
        
        //telemetry.addData("" + robot.color.red(), "" + robot.color.green(), "" + robot.color.blue());
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
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRear.getCurrentPosition() + (int)(leftRearInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRear.getCurrentPosition() + (int)(rightRearInches * COUNTS_PER_INCH);
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
                telemetry.addData("Distance:", distance.getDistance(DistanceUnit.CM));
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
}