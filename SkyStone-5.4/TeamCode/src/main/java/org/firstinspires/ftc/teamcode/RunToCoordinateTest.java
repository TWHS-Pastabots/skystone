package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RunToCoordinateTest extends LinearOpMode {

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

        Positioning positioning = new Positioning(robot.leftEnc, robot.rightEnc, robot.horizEnc, 50, robot.imu, 0.0, 0, 0);
        Thread positionThread = new Thread(positioning);
        positionThread.start();
        telemetry.addData("Status", "Started Thread");
        telemetry.update();

        driveToPosition(10, 20, 0.4, 10, positioning);

        positioning.stop();
    }

    public void driveToPosition(double targetX, double targetY, double speed, double timeoutS, Positioning positioning){
        telemetry.addData("Status", "Begun Drive Method");
        telemetry.update();
        double xDistance = targetX - positioning.getX();
        double yDistance = targetY - positioning.getY();
        double distance = Math.hypot(xDistance, yDistance);
        double movementAngle;

        double fLeft;
        double fRight;
        double rLeft;
        double rRight;

        runTime.reset();
        while(runTime.seconds() < timeoutS && distance > 1.0){
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

    }

    private double calcOtherPower(double firstPower, double firstAngleDegrees, double otherAngleDegrees, double finalAngleDegrees ){
        double th1 = Math.toRadians(firstAngleDegrees);
        double th2 = Math.toRadians(otherAngleDegrees);
        double thf = Math.toRadians(finalAngleDegrees);

        //Calculate the power using a formula that was derived using vector addition
        return ( (firstPower * Math.cos(th1) * Math.tan(thf)) - (firstPower * Math.sin(th1)) ) / ( Math.sin(th2) - (Math.cos(th2) * Math.tan(thf)) );
    }

}
