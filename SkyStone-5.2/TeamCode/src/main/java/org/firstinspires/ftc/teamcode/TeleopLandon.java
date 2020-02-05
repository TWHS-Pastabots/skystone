package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp
public class TeleopLandon extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private DcMotor motor_lift;
    private DcMotor motor_encoder;
    private Servo servo_gripLeft;
    private Servo servo_gripRight;
    private Servo servo_grabber;
    private Servo servo_platformRight;
    private Servo servo_platformLeft;
    private Servo servo_cup;
    private DcMotor motor_tape;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean fineControlMode;
    private boolean platformLowered = false;
    private boolean cupIsGo = false;
    private ElapsedTime aTimer = new ElapsedTime();
    private ElapsedTime aTimer2 = new ElapsedTime();
    private ElapsedTime yTimer = new ElapsedTime();
    private ElapsedTime xTimer = new ElapsedTime();
    private ElapsedTime dpadTimer = new ElapsedTime();
    private ElapsedTime cupTimer = new ElapsedTime();
    private ArrayList<String> labelList = new ArrayList<>();
    private ArrayList<String> dataList = new ArrayList<>();
    private HashMap<Integer, Double> armPositions = new HashMap<>();
    private int armPosition = 4;


    @Override
    public void runOpMode() {
        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        motor_lift = hardwareMap.get(DcMotor.class, "motor_lift");
        motor_encoder = hardwareMap.get(DcMotor.class, "motor_encoder");
        motor_tape = hardwareMap.get(DcMotor.class, "motor_tape");
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
        motor_tape.setDirection(DcMotor.Direction.FORWARD);

        servo_cup.setPosition(0.0);
        servo_grabber.resetDeviceConfigurationForOpMode();
        servo_gripRight.resetDeviceConfigurationForOpMode();
        servo_gripLeft.resetDeviceConfigurationForOpMode();
        servo_platformLeft.resetDeviceConfigurationForOpMode();
        servo_platformRight.resetDeviceConfigurationForOpMode();

        armPositions.put(1, 1.0);
        armPositions.put(2, 0.6);
        armPositions.put(3, 0.5);
        armPositions.put(4, 0.3);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double centerPower;
            double liftPower = 0;
            double tapePower = 0;

            double controlCoeff = 1;

            if (gamepad1.a && aTimer.milliseconds() > 1000) {
                fineControlMode = !fineControlMode;
                aTimer.reset();
            }

            if(gamepad2.start){
                tapePower = 1;
            }
            else if(gamepad2.back){
                tapePower = -1;
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
            if(turnLeft && !turnRight){
                leftPower = 0.75;
                rightPower = -0.75;
            }else if(turnRight && !turnLeft){
                rightPower = 0.75;
                leftPower = -0.75;
            }

            if(gamepad2.dpad_up && armPosition < 4 && dpadTimer.milliseconds() > 500) {
                armPosition++;
                dpadTimer.reset();
            }
            else if(gamepad2.dpad_down && armPosition > 1 && dpadTimer.milliseconds() > 500) {
                armPosition--;
                dpadTimer.reset();
            }


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

            if(gamepad2.x && xTimer.milliseconds() > 1000){
                cupIsGo = !cupIsGo;
                xTimer.reset();
            }

            if(gamepad2.right_trigger > 0.0){
                liftPower = gamepad2.right_trigger;
            }
            else if(gamepad2.left_trigger > 0.0){
                liftPower = -1 * gamepad2.left_trigger;
            }



            motor_left.setPower(leftPower * controlCoeff);
            motor_right.setPower(rightPower * controlCoeff);
            motor_center.setPower(centerPower * controlCoeff);
            motor_lift.setPower(liftPower);
            motor_tape.setPower(tapePower);
            if(platformLowered){
                servo_platformRight.setPosition(0.9);
                servo_platformLeft.setPosition(0.1);
            }
            else{
                servo_platformRight.setPosition(0.0);
                servo_platformLeft.setPosition(1.0);
            }
            if(cupIsGo){
                servo_cup.setPosition(1.0);
            }
            else{
                servo_cup.setPosition(0.0);
            }
            servo_grabber.setPosition(armPositions.get(armPosition));


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Fine Control Mode:", "" + fineControlMode);
            telemetry.addData("Encoder value: ", motor_encoder.getCurrentPosition());
            telemetry.update();
        }
    }

}
