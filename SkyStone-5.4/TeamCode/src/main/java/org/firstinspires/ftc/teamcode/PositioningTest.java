package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Cam and Matthew on .
 * Example OpMode that runs the Positioning thread and accesses the (x, y, theta) coordinate values
 */

public class PositioningTest extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode(){
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

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", positioning.getX());
            telemetry.addData("Y Position", positioning.getY());
            telemetry.addData("Orientation (Degrees)", positioning.getOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        positioning.stop();
    }
}
