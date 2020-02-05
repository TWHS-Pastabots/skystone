package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

@TeleOp
public class TeleopFWEPA extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private DcMotor motor_lift;
    private DcMotor motor_encoder;
    private DcMotor motor_encoderCenter;
    private Servo servo_gripLeft;
    private Servo servo_gripRight;
    private Servo servo_grabber;
    private Servo servo_platformRight;
    private Servo servo_platformLeft;
    private Servo servo_cup;
    private DcMotor motor_tape;
    private BNO055IMU imu;
    double posX;
    double posY;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean fineControlMode;
    private boolean platformLowered = false;
    private boolean cupIsGo = false;
    private ElapsedTime aTimer = new ElapsedTime();
    private ElapsedTime aTimer2 = new ElapsedTime();
    private ElapsedTime yTimer = new ElapsedTime();
    private ElapsedTime xTimer = new ElapsedTime();
    private ElapsedTime cupTimer = new ElapsedTime();
    private ArrayList<String> labelList = new ArrayList<>();
    private ArrayList<String> dataList = new ArrayList<>();

    static final double     ENCODER_COUNTS_PER_MOTOR_REV    = 2400; //THIS CHANGED
    static final double     SMALL_WHEEL_DIAMETER_INCHES   = 2.3622 ;     // For figuring circumference
    static final double     ENCODER_COUNTS_PER_INCH         = (ENCODER_COUNTS_PER_MOTOR_REV) / (SMALL_WHEEL_DIAMETER_INCHES * 3.1416);





    @Override
    public void runOpMode() {
        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        motor_lift = hardwareMap.get(DcMotor.class, "motor_lift");
        motor_encoder = hardwareMap.get(DcMotor.class, "motor_encoder");
        motor_encoderCenter = hardwareMap.get(DcMotor.class, "motor_encoderCenter");
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

        servo_cup.resetDeviceConfigurationForOpMode();
        servo_cup.setPosition(1.0);
        servo_grabber.resetDeviceConfigurationForOpMode();
        servo_gripRight.resetDeviceConfigurationForOpMode();
        servo_gripLeft.resetDeviceConfigurationForOpMode();
        servo_platformLeft.resetDeviceConfigurationForOpMode();
        servo_platformRight.resetDeviceConfigurationForOpMode();

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode                = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        double lastPos = motor_encoder.getCurrentPosition();
        double lastPosCenter = motor_encoderCenter.getCurrentPosition();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double centerPower;
            double liftPower = 0;
            double tapePower = 0;

            calcPos(lastPos, lastPosCenter);

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

            if(gamepad2.dpad_up) {
                servo_grabber.setPosition(0.3);
            }
            else if(gamepad2.dpad_down) {

                servo_grabber.setPosition(1.0);

            }
            else if(gamepad2.a) {
                servo_grabber.setPosition(0.6);

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
                servo_cup.setPosition(0.0);
            }
            else{
                servo_cup.setPosition(1.0);
            }

            lastPos = motor_encoder.getCurrentPosition();
            lastPosCenter = motor_encoderCenter.getCurrentPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X Position:", "%4.2f:", posX);
            telemetry.addData("Y Position:", "%4.2f:", posY);
            telemetry.addData("Angle:", "%4.2f:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Fine Control Mode:", "" + fineControlMode);
            telemetry.addData("Encoder value: ", motor_encoder.getCurrentPosition());
            telemetry.update();
        }
    }

    private void calcPos(double lastEncoderPos, double lastCenterEncoderPos){
        double encoderDelta;
        double centerEncoderDelta;
        encoderDelta = motor_encoder.getCurrentPosition() - lastEncoderPos;
        centerEncoderDelta = motor_encoderCenter.getCurrentPosition() - lastCenterEncoderPos;
        posX += (encoderDelta / ENCODER_COUNTS_PER_INCH) * Math.sin(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        posY += (encoderDelta / ENCODER_COUNTS_PER_INCH) * Math.cos(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        posX += (centerEncoderDelta / ENCODER_COUNTS_PER_INCH) * Math.cos(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        posY += (centerEncoderDelta / ENCODER_COUNTS_PER_INCH) * Math.sin(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

}
