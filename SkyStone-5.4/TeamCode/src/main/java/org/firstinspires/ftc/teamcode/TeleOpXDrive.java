package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp
public class TeleOpXDrive extends LinearOpMode {

    private DcMotor motor_fLeft;
    private DcMotor motor_fRight;
    private DcMotor motor_bLeft;
    private DcMotor motor_bRight;


    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        motor_fLeft = hardwareMap.get(DcMotor.class, "left_front");
        motor_fRight = hardwareMap.get(DcMotor.class, "right_front");
        motor_bLeft = hardwareMap.get(DcMotor.class, "left_rear");
        motor_bRight = hardwareMap.get(DcMotor.class, "right_rear");



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor_fLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_fRight.setDirection(DcMotor.Direction.FORWARD);
        motor_bLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_bRight.setDirection(DcMotor.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drivePower;
            double fLeftPower;
            double fRightPower;
            double bLeftPower;
            double bRightPower;

            double forwardBias = 0.5;
            double centerBias = 1 - forwardBias;

            //Handle Driving Forward
            double drive = gamepad1.left_stick_y;
            drivePower = drive * forwardBias;

            //Handle Driving Sideways
            double cDrive = gamepad1.left_stick_x;
            fLeftPower = drivePower + (cDrive * centerBias);
            fRightPower = drivePower + (cDrive * centerBias * -1.0);
            bLeftPower = drivePower + (cDrive * centerBias * -1.0);
            bRightPower = drivePower + (cDrive * centerBias);

            //Handle turning
            double leftBias = gamepad1.right_trigger;
            double rightBias = gamepad1.left_trigger;
            if(drive == 0 && cDrive == 0) {
                if (leftBias > 0 && !(rightBias > 0)) {
                    //turn right
                    fLeftPower = leftBias;
                    fRightPower = -1.0 * leftBias;
                    bLeftPower = leftBias;
                    bRightPower = -1.0 * leftBias;
                } else if (rightBias > 0 && !(leftBias > 0)) {
                    //turn left
                    fLeftPower = -1.0 * rightBias;
                    fRightPower = rightBias;
                    bLeftPower = -1.0 * rightBias;
                    bRightPower = rightBias;
                }
            }
            else if(leftBias > 0 || rightBias > 0){
                if (leftBias > 0 && !(rightBias > 0)){
                    //turn right
                    double proportionMultiplier = 1.0 + leftBias;
                    fRightPower /= proportionMultiplier;
                    bRightPower /= proportionMultiplier;
                }
                else if (rightBias > 0 && !(leftBias > 0)) {
                    //turn left
                    double proportionMultiplier = 1.0 + rightBias;
                    fLeftPower /= proportionMultiplier;
                    bLeftPower /= proportionMultiplier;
                }

            }

            //Give power to the motors
            motor_fLeft.setPower(fLeftPower);
            motor_fRight.setPower(fRightPower);
            motor_bLeft.setPower(bLeftPower);
            motor_bRight.setPower(bRightPower);

            //Provide telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", drivePower, rightPower);
            telemetry.update();
        }
    }

}
