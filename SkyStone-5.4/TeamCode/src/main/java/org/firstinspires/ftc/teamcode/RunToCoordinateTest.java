package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous
public class RunToCoordinateTest extends LinearOpMode  {

    private final double ANGLE_THRESHOLD = 3.0;
    private final double TURN_SPEED = 0.4;
    private final double DRIVE_SPEED = 0.4;
    private final double START_X = 0.0;
    private final double START_Y = 0.0;
    private final double START_ORIENTATION = 90.0;
    private final int THREAD_SLEEP_DELAY = 50;

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runTime= new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        Positioning positioning = new Positioning(robot.leftEnc, robot.rightEnc, robot.horizEnc, THREAD_SLEEP_DELAY, robot.imu, START_ORIENTATION, START_X, START_Y);
        Thread positionThread = new Thread(positioning);
        positionThread.start();
        telemetry.addData("Status", "Started Thread");
        telemetry.update();

        //START OF DRIVING

        driveToPosition(0.0, 35.0, DRIVE_SPEED, 1.0, 6.0, 10, positioning);
        robot.leftIn.setPower(-.7);
        robot.rightIn.setPower(-.7);
        driveToPosition(4.0, 35.0, DRIVE_SPEED, 0.5, 2.0, 10, positioning);
        sleep(1000);
        robot.leftIn.setPower(0);
        robot.rightIn.setPower(0);

        telemetry.addData("Status:", "Finished Driving");
        telemetry.update();

        positioning.stop();
    }

    public void driveToPosition(double targetX, double targetY, double speed, double rampUpTimeS, double rampDownDistance, double timeoutS, Positioning positioning){
        double xDistance = targetX - positioning.getX();
        double yDistance = targetY - positioning.getY();
        double distance = Math.hypot(xDistance, yDistance);
        double movementAngle;
        double startOrientation = positioning.getOrientation();

        double fLeft;
        double fRight;
        double rLeft;
        double rRight;
        int successCounter = 0;

        runTime.reset();
        while(runTime.seconds() < timeoutS && distance > 0.5){
            xDistance = targetX - positioning.getX();
            yDistance = targetY - positioning.getY();
            distance = Math.hypot(xDistance, yDistance);
            movementAngle = Math.toDegrees(Math.atan2(xDistance, yDistance)) - positioning.getOrientation();

            //Convert the movement angle, which is relative to the y-axis to be relative to the x-axis for future calculations
            double angleXAxis = 90 - movementAngle;
            if(angleXAxis > 180)
                angleXAxis -= 360;
            else if(angleXAxis < -180)
                angleXAxis += 360;

            fLeft = 0;
            fRight = 0;
            rLeft = 0;
            rRight = 0;

            //Calculate the speeds based on the quadrant the robot is traveling to
            if(angleXAxis > 0 && angleXAxis < 90) {  //First Quadrant
                fLeft = speed;
                fRight = calcOtherPower(fLeft, 45.0, 135.0, angleXAxis);
                rRight = speed;
                rLeft = calcOtherPower(rRight, 45.0, 135.0, angleXAxis);
            }
            else if(angleXAxis > 90 && angleXAxis < 180){  //Second Quadrant
                fRight = speed;
                fLeft = calcOtherPower(fRight, 135.0, 45.0, angleXAxis);
                rLeft = speed;
                rRight = calcOtherPower(rLeft, 135.0, 45.0, angleXAxis);
            }
            else if(angleXAxis > -180 && angleXAxis < -90){ //Third Quadrant
                fLeft = -speed;
                fRight = calcOtherPower(fLeft, 45.0, 135.0, angleXAxis);
                rRight = -speed;
                rLeft = calcOtherPower(rRight, 45.0, 135.0, angleXAxis);
            }
            else if(angleXAxis < 0 && angleXAxis > -90){ //Fourth Quadrant
                fRight = -speed;
                fLeft = calcOtherPower(fRight, 135.0, 45.0, angleXAxis);
                rLeft = -speed;
                rRight = calcOtherPower(rLeft, 135.0, 45.0, angleXAxis);
            }
            else if( angleXAxis == 0.0){
                fLeft = speed;
                fRight = -speed;
                rLeft = -speed;
                rRight = speed;
            }
            else if(angleXAxis == 90.0){
                fLeft = speed;
                fRight = speed;
                rLeft = speed;
                rRight = speed;
            }
            else if( angleXAxis == 180.0 || angleXAxis == -180.0){
                fLeft = -speed;
                fRight = speed;
                rLeft = speed;
                rRight = -speed;
            }
            else if(angleXAxis == -90.0){
                fLeft = -speed;
                fRight = -speed;
                rLeft = -speed;
                rRight = -speed;
            }


            //Ramp up the motor powers
            if(rampUpTimeS > 0) {
                fLeft = fLeft * (runTime.seconds() / rampUpTimeS);
                fRight = fRight * (runTime.seconds() / rampUpTimeS);
                rLeft = rLeft * (runTime.seconds() / rampUpTimeS);
                rRight = rRight * (runTime.seconds() / rampUpTimeS);
            }

            //Ramp down the motor powers
            if(distance < rampDownDistance){
                double rampDownPercent = distance / rampDownDistance;
                if(rampDownPercent < 0.2)
                    rampDownPercent = 0.2;
                fLeft = fLeft * (rampDownPercent);
                fRight = fRight * (rampDownPercent);
                rLeft = rLeft * (rampDownPercent);
                rRight = rRight * (rampDownPercent);
            }



            //Give powers to the wheels
            robot.leftFront.setPower(fLeft);
            robot.rightFront.setPower(fRight);
            robot.leftRear.setPower(rLeft);
            robot.rightRear.setPower(rRight);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", positioning.getX());
            telemetry.addData("Y Position", positioning.getY());
            telemetry.addData("Orientation (Degrees)", positioning.getOrientation());
            telemetry.update();
        }

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

        //Turn to the orientation the move was started at
        turn(startOrientation, TURN_SPEED, positioning);

    }

    private void turn(double targetAngle, double turnSpeed, Positioning positioning){

        double angleError;
        int successCounter = 0;

        runTime.reset();
        while(successCounter < 5){

            angleError = targetAngle - positioning.getOrientation();
            double signedTurnSpeed = turnSpeed * Math.signum(angleError);

            //Ramp down
            if(Math.abs(angleError) < 20)
                signedTurnSpeed *= 0.25;
            else if(Math.abs(angleError) < 40)
                signedTurnSpeed *= 0.4;
            else if(Math.abs(angleError) < 60)
                signedTurnSpeed *= 0.5;
            else if(Math.abs(angleError) < 80)
                signedTurnSpeed *= 0.75;

            robot.leftFront.setPower(signedTurnSpeed);
            robot.rightFront.setPower(-signedTurnSpeed);
            robot.leftRear.setPower(signedTurnSpeed);
            robot.rightRear.setPower(-signedTurnSpeed);

            if(Math.abs(angleError) < ANGLE_THRESHOLD && runTime.milliseconds() > 50) {
                successCounter++;
                runTime.reset();
            }
            else if(Math.abs(angleError) > ANGLE_THRESHOLD)
                successCounter = 0;

            telemetry.addData("Successes:", successCounter);
            telemetry.addData("Error", Math.abs(angleError));
            telemetry.update();
        }

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
    }

    private double calcOtherPower(double firstPower, double firstAngleDegrees, double otherAngleDegrees, double finalAngleDegrees ){
        double th1 = Math.toRadians(firstAngleDegrees);
        double th2 = Math.toRadians(otherAngleDegrees);
        double thf = Math.toRadians(finalAngleDegrees);

        //Calculate the power using a formula that was derived using vector addition
        return ( (firstPower * Math.cos(th1) * Math.tan(thf)) - (firstPower * Math.sin(th1)) ) / ( Math.sin(th2) - (Math.cos(th2) * Math.tan(thf)) );
    }



}
