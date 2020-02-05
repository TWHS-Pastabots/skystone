package DeprecatedCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp
@Disabled
public class TestBetterTurnTeleOp extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private Servo servo_gripLeft;
    private Servo servo_gripRight;
    private Servo servo_grabber;
    private Servo servo_platformRight;
    private Servo servo_platformLeft;
    private Servo servo_cup;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean fineControlMode;
    private boolean platformLowered = false;
    private boolean cupIsGo = false;
    private ElapsedTime aTimer = new ElapsedTime();
    private ElapsedTime aTimer2 = new ElapsedTime();
    private ElapsedTime yTimer = new ElapsedTime();
    private ElapsedTime xTimer = new ElapsedTime();
    private ElapsedTime cupTimer = new ElapsedTime();

    private final double TURN_MODIFIER = 0.1;


    @Override
    public void runOpMode() {
        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        servo_cup = hardwareMap.get(Servo.class, "servo_cup");
        servo_grabber = hardwareMap.get(Servo.class, "servo_grabber");
        servo_gripLeft = hardwareMap.get(Servo.class, "servo_gripLeft");
        servo_gripRight = hardwareMap.get(Servo.class, "servo_gripRight");
        servo_platformRight = hardwareMap.get(Servo.class, "servo_platformRight");
        servo_platformLeft = hardwareMap.get(Servo.class, "servo_platformLeft");

        fineControlMode = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_center.setDirection(DcMotor.Direction.FORWARD);

        servo_cup.resetDeviceConfigurationForOpMode();
        servo_grabber.resetDeviceConfigurationForOpMode();
        servo_gripRight.resetDeviceConfigurationForOpMode();
        servo_gripLeft.resetDeviceConfigurationForOpMode();
        servo_platformLeft.resetDeviceConfigurationForOpMode();
        servo_platformRight.resetDeviceConfigurationForOpMode();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double centerPower;
            double cupPower;

            double controlCoeff = 1;

            if (gamepad1.a && aTimer.milliseconds() > 1000) {
                fineControlMode = !fineControlMode;
                aTimer.reset();
            }

            if(fineControlMode)
                controlCoeff = 0.5;


            double drive = gamepad1.left_stick_y;
            double center = gamepad1.right_stick_x;
            leftPower = drive;
            rightPower = drive;
            centerPower = center;
            boolean turnLeft = gamepad1.left_bumper;
            boolean turnRight = gamepad1.right_bumper;
            if(drive > 0) {
                if (turnLeft && !turnRight) {
                    leftPower -= TURN_MODIFIER;
                    rightPower += TURN_MODIFIER;
                } else if (turnRight && !turnLeft) {
                    rightPower -= TURN_MODIFIER;
                    leftPower += TURN_MODIFIER;
                }
                else{
                    leftPower = drive;
                    rightPower = drive;
                }
                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
                if (max > 1.0)
                {
                    leftPower /= max;
                    rightPower /= max;
                }
            }
            else{
                if(turnLeft && !turnRight){
                    leftPower = 0.75;
                    rightPower = -0.75;
                }else if(turnRight && !turnLeft){
                    rightPower = 0.75;
                    leftPower = -0.75;
                }
            }

            if(gamepad2.dpad_up)
                servo_grabber.setPosition(0.25);
            else if(gamepad2.dpad_down)
                servo_grabber.setPosition(1.0);
            else if(gamepad2.a)
                servo_grabber.setPosition(0.33);

            if(gamepad2.left_bumper) {
                servo_gripRight.setPosition(0.0);
                servo_gripLeft.setPosition(1.0);
            }
            else if(gamepad2.right_bumper) {
                servo_gripRight.setPosition(1.0);
                servo_gripLeft.setPosition(0.0);
            }

            if(gamepad2.y && yTimer.milliseconds() > 1000){
                platformLowered = !platformLowered;
                yTimer.reset();
            }

            if(gamepad2.x && xTimer.milliseconds() > 5000){
                cupIsGo = true;
                cupTimer.reset();
            }



            motor_left.setPower(leftPower * controlCoeff);
            motor_right.setPower(rightPower * controlCoeff);
            motor_center.setPower(centerPower * controlCoeff);
            if(platformLowered){
                servo_platformRight.setPosition(1.0);
                servo_platformLeft.setPosition(0.0);
            }
            else{
                servo_platformRight.setPosition(0.0);
                servo_platformLeft.setPosition(1.0);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Fine Control Mode:", "" + fineControlMode);
            telemetry.update();
        }
    }

}
