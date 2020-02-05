package DeprecatedCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class OldBasicTeleOp extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private DcMotor motor_intake_left;
    private DcMotor motor_intake_right;
    private CRServo servo_arm;
    private Servo servo_grip;
    private Servo servo_hook;
    private CRServo servo_intake_left;
    private CRServo servo_intake_right;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean fineControlMode;
    private ElapsedTime aTimer = new ElapsedTime();
    private ElapsedTime aTimer2 = new ElapsedTime();


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
        servo_intake_left = hardwareMap.get(CRServo.class, "servo_intake_left");
        servo_intake_right = hardwareMap.get(CRServo.class, "servo_intake_right");

        fineControlMode = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_center.setDirection(DcMotor.Direction.REVERSE);
        motor_intake_left.setDirection(DcMotor.Direction.REVERSE);
        motor_intake_right.setDirection(DcMotor.Direction.FORWARD);

        servo_arm.resetDeviceConfigurationForOpMode();
        servo_grip.resetDeviceConfigurationForOpMode();
        servo_hook.resetDeviceConfigurationForOpMode();
        servo_intake_left.resetDeviceConfigurationForOpMode();
        servo_intake_right.resetDeviceConfigurationForOpMode();
        servo_hook.setPosition(0);
        servo_grip.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double centerPower;
            double intakePower;
            double armServoPower;
            double servoIntakePower;

            double controlCoeff = 1;

            if (gamepad1.a && aTimer.milliseconds() > 1000) {
                fineControlMode = !fineControlMode;
                aTimer.reset();
            }

            if(fineControlMode)
                controlCoeff = 0.5;


            double drive = -1*gamepad1.left_stick_y; //I needed to reverse the direction so I did this
            double center = -1*gamepad1.left_stick_x;
            leftPower = drive;
            rightPower = drive+0.1;
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

            intakePower = 0;
            if(gamepad2.left_bumper && !gamepad2.right_bumper)
                intakePower = 1;
            if(gamepad2.right_bumper && !gamepad2.left_bumper)
                intakePower = -1;

            servoIntakePower = 0;
            if(gamepad2.right_trigger == 1 && gamepad2.left_trigger == 0)
                servoIntakePower = 1;
            if(gamepad2.right_trigger == 0 && gamepad2.left_trigger == 1)
                servoIntakePower = -1;

            armServoPower = 0;
            if(gamepad2.a){
                armServoPower = 1;
            }
            else if(gamepad2.b){
                armServoPower = -1;
            }

            if(gamepad2.dpad_left)
                servo_grip.setPosition(0);
            else if(gamepad2.dpad_right)
                servo_grip.setPosition(1);

            if(gamepad2.dpad_down)
                servo_hook.setPosition(0.52);
            else if(gamepad2.dpad_up)
                servo_hook.setPosition(0);

            motor_left.setPower(leftPower * controlCoeff);
            motor_right.setPower(rightPower * controlCoeff);
            motor_center.setPower(centerPower * controlCoeff);
            motor_intake_left.setPower(intakePower);
            motor_intake_right.setPower(intakePower);
            servo_arm.setPower(armServoPower);
            servo_intake_right.setPower(-1*servoIntakePower);
            servo_intake_left.setPower(servoIntakePower);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Fine Control Mode:", "" + fineControlMode);
            telemetry.addData("Servo Power:", "" + servo_arm.getPower());
            telemetry.addData("Claw Servo Pos", ""+servo_grip.getPosition());
            telemetry.addData("Hook Servo Pos", ""+servo_hook.getPosition());
            telemetry.update();
        }
    }
}
