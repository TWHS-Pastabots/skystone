package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //Odometry encoder wheels
    private DcMotor encoder_left;
    private DcMotor encoder_right;
    private DcMotor encoder_horizontal;

    private DistanceSensor distance;
    private TouchSensor touch;
    DigitalChannel magnet;

    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime= new ElapsedTime();
    double slowCon = 1.0;
    int pos = 0;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 1141.9488791276;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String leftEncoderName = "left_rear", rightEncoderName ="right_rear", horizontalEncoderName = "lift_motor";

    @Override
    public void runOpMode() throws InterruptedException {

        //Assign the hardware map to the odometry wheels
        encoder_left = hardwareMap.dcMotor.get(leftEncoderName);
        encoder_right = hardwareMap.dcMotor.get(rightEncoderName);
        encoder_horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);
        //horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        encoder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        encoder_left.setDirection(DcMotorSimple.Direction.FORWARD);
        encoder_right.setDirection(DcMotorSimple.Direction.REVERSE);
        //horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        encoder_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder_horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(encoder_left, encoder_right, encoder_horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

            double G1leftStickX = -gamepad1.right_stick_x;
            double G1leftStickY = -gamepad1.right_stick_y;
            double turnCon = gamepad1.left_stick_x;
            boolean G1a = gamepad1.a;
            boolean G1b = gamepad1.b;
            boolean G2a = gamepad2.a;
            boolean G2b = gamepad2.b;
            boolean G2x = gamepad2.x;
            boolean G2y = gamepad2.y;
            boolean G1y = gamepad1.y;
            double G2leftStickY = -gamepad2.left_stick_y;
            boolean G2rb = gamepad2.right_bumper;
            boolean G2lb = gamepad2.left_bumper;
            boolean G1down = gamepad1.dpad_down;
            boolean G1up = gamepad1.dpad_up;
            boolean G2up = gamepad2.dpad_up;
            boolean G2down = gamepad2.dpad_down;
            boolean G2left = gamepad2.dpad_left;
            boolean G2right = gamepad2.dpad_right;
            // example how to use buttons
            double radius = Math.hypot( G1leftStickX, G1leftStickY);
            double ang = Math.atan2( G1leftStickY, G1leftStickX) - Math.PI/4;
            double v1 = radius * Math.cos(ang) + turnCon;
            double v2 = radius * Math.sin(ang) - turnCon;
            double v3 = radius * Math.sin(ang) + turnCon;
            double v4 = radius * Math.cos(ang) - turnCon;

            // Sets power of motor, spins wheels

            if (G1b){
                slowCon = .3;

            }
            if (G1a){
                slowCon = 1.0;
            }

            // intake code
            if(gamepad1.right_bumper){
                robot.leftIn.setPower(-.7);
                robot.rightIn.setPower(-.7);
            }
            else if(gamepad1.left_bumper){
                robot.leftIn.setPower(.7);
                robot.rightIn.setPower(.7);
            }
            else {
                robot.leftIn.setPower(0);
                robot.rightIn.setPower(0);
            }

            // lift code
            if (G2up && magnet.getState())
            {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-0.5);
                pos = robot.liftMotor.getCurrentPosition();
            }
            else if (G2down && !touch.isPressed())
            {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(0.2);
                pos = robot.liftMotor.getCurrentPosition();
            }
            else
            {
                robot.liftMotor.setTargetPosition(pos);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(1);
                telemetry.addData("Staying at:", robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            //servos
            if (G2rb)
            {
                robot.leftH.setPosition(0);
                robot.rightH.setPosition(1);
            }
            else if (G2lb)
            {
                robot.leftH.setPosition(1);
                robot.rightH.setPosition(0);
            }
            // claw
            if (G2a)
            {
                robot.claw.setPosition(1);
            }
            else if (G2b)
            {
                robot.claw.setPosition(0);
            }

            if(G2right)
            {
                robot.clawT.setPosition(0);
            }
            else if (G2left)
            {
                robot.clawT.setPosition(1);
            }

        /*if (G1leftStickY>.9){
            v1=1;
            v2=1;
            v3=1;
            v4=1;
        }
        if (G1leftStickY<-.9){
            v1=-1;
            v2=-1;
            v3=-1;
            v4=-1;
        }

        if (G1leftStickY<-.9){
            v1=-1;
            v2=-1;
            v3=-1;
            v4=-1;
        }
        if (G1leftStickY>.9){
            v1=1;
            v2=1;
            v3=1;
            v4=1;
        }
        if (G1leftStickX>.9){
            v1=1;
            v2=-1;
            v3=-1;
            v4=1;
        }
        if (G1leftStickX<-.9){
            v1=-1;
            v2=1;
            v3=1;
            v4=-1;
        }

         */

            robot.leftFront.setPower(v1*slowCon );
            robot.rightFront.setPower(v2*slowCon );
            robot.leftRear.setPower(v3*slowCon );
            robot.rightRear.setPower(v4*slowCon );

            telemetry.addData("Powers:", v1);
            telemetry.addData("", v2);
            telemetry.addData("", v3);
            telemetry.addData("", v4);
            telemetry.addData("Magnet:", magnet.getState());
            //telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}