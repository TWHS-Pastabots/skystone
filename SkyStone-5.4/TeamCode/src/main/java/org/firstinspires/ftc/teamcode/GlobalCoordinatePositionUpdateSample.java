package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}