package DeprecatedCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class TimeAuton extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private DcMotor motor_intake_left;
    private DcMotor motor_intake_right;
    private CRServo servo_arm;
    private Servo servo_grip;
    private Servo servo_hook;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime aTimer = new ElapsedTime();
    private ElapsedTime aTimer2 = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double    CENTER_SPEEED = 1.0;


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
        servo_grip.setPosition(0);
        servo_hook.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //drive forward 1 sec
        motor_left.setPower(FORWARD_SPEED);
        motor_right.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Forward:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //drive sideways 1 second
        motor_center.setPower(CENTER_SPEEED);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1.0)){
            telemetry.addData("Sidways:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //drive forward 1.3 sec
        motor_left.setPower(FORWARD_SPEED);
        motor_right.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Forward:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Move hook down and wait 1 sec
        servo_hook.setPosition(0.52);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)){
            telemetry.addData("Hooking:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //drive backward 2.3 sec
        motor_left.setPower(-1*FORWARD_SPEED);
        motor_right.setPower(-1*FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Backward:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        /**
        //Spin right for 1.3 seconds
        motor_left.setPower(TURN_SPEED);
        motor_right.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Turning Right:", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }**/

    }
}
