package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Last updated by Cam and Matthew on 02102020
 *
 * This file encapsulates all of the robot's hardware into one class.
 *
 * Once you have instantiated a piece of hardware in the RobotHardware class,
 * you can pull from it in other existing classes.
 *
 * Essentially just makes it easier to access motors, servos, sensors, etc. without having to
 * initialize all of them each time you make a new class.
 *
 */
public class RobotHardware {

    /* Declare OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotor tape = null;
    public DcMotor leftIn = null;
    public DcMotor rightIn = null;
    public DcMotor liftMotor = null;
    public DcMotor leftEnc = null;
    public DcMotor rightEnc = null;
    public DcMotor horizEnc = null;

    public Servo leftH;
    public Servo rightH;
    public Servo claw;
    public Servo clawT;
    public Servo servo_blockPush;
    public CRServo capstone;

    public DistanceSensor blockInSensor;
    public DistanceSensor armHeightDistance;
    public DistanceSensor blockDistance;
    public DistanceSensor frontDistance;
    public DistanceSensor leftDistance;
    public ColorSensor colorSensor;
    public BNO055IMU imu = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "left_front");
        horizEnc = hwMap.get(DcMotor.class, "tape_motor");
        tape = hwMap.get(DcMotor.class, "tape_motor");
        leftRear = hwMap.get(DcMotor.class, "left_rear");
        leftEnc = hwMap.get(DcMotor.class, "left_rear");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        rightRear = hwMap.get(DcMotor.class, "right_rear");
        rightEnc = hwMap.get(DcMotor.class, "right_rear");
        leftIn = hwMap.get(DcMotor.class, "left_intake");
        rightIn = hwMap.get(DcMotor.class, "right_intake");
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");
        leftH = hwMap.get(Servo.class, "left_hook");
        rightH = hwMap.get(Servo.class, "right_hook");
        claw = hwMap.get(Servo.class, "claw");
        clawT = hwMap.get(Servo.class, "claw_turn");
        imu = hwMap.get(BNO055IMU.class, "imu");
        blockInSensor = hwMap.get(DistanceSensor.class, "color");
        colorSensor = hwMap.get(ColorSensor.class, "color");
        servo_blockPush = hwMap.get(Servo.class, "servo_blockPush");
        capstone = hwMap.get(CRServo.class, "capstone");
        armHeightDistance = hwMap.get(DistanceSensor.class, "armHeightDistance");
        blockDistance = hwMap.get(DistanceSensor.class, "blockDistance");
        frontDistance = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistance = hwMap.get(DistanceSensor.class, "leftDistance");

        // Motor direction is FORWARD by default
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftIn.setDirection(DcMotor.Direction.REVERSE);
        rightIn.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        tape.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftIn.setPower(0);
        rightIn.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tape.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        servo_blockPush.setPosition(1.0);
    }
}
