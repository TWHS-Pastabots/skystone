package DeprecatedCode;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous
@Disabled
public class NewAuton extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private Servo servo_gripLeft;
    private Servo servo_gripRight;
    private Servo servo_grabber;
    private Servo servo_platformRight;
    private Servo servo_platformLeft;
    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();


    static final double     COREHEX_COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Core Hex Motor Encoder
    static final double     HEX140_COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // Gear ratio maybe 0.375
    static final double     CENTER_GEAR_REDUCTION    = 1 ;     // Gear ratio
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final double     SMALL_WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     DRIVE_COUNTS_PER_INCH         = (COREHEX_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     CENTER_COUNTS_PER_INCH = (HEX140_COUNTS_PER_MOTOR_REV * CENTER_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;
    static final double     HEADING_THRESHOLD       = 0.5 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "AT27Zpn/////AAABma7sU1s4XEtftm65yZXkRCWM9i6m/6wUIGvUsFBSzT7L9tlAhl6/ckO7gKvgbCNwRbANbj561XOWx1QuHiRnbSz3JeukZwsEkdkEANU7lroQ30T/QadU3YNQlSL09JazvYxX2UkXiVBNnadlECH+4/mBqUNGCHsKnyH5hVArtcjRsSSc3B5GuFqqO8zD35HW3ot0pPMsoFuJA7XyXhx9N2DcVxGdAZWhtqLQw91ZymFpOAT6YIxQcQfqu2BOuvU0y7Z2wjlE9BvVD/BXFNQreloZlan8LZT+NTqinwlSEwjvL6Y4v5VDcoAR9xRHBmzKlin75QH/WoRR+S1Z3pF6lUahubM9/ZnGFb/WSMpOjiEi";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private List<VuforiaTrackable> allTrackables;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";
    private TFObjectDetector tfod;

    private double xPos;
    private double yPos;

    @Override
    public void runOpMode() {
        runtime.reset();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");
        servo_grabber = hardwareMap.get(Servo.class, "servo_grabber");
        servo_gripLeft = hardwareMap.get(Servo.class, "servo_gripLeft");
        servo_gripRight = hardwareMap.get(Servo.class, "servo_gripRight");
        servo_platformRight = hardwareMap.get(Servo.class, "servo_platformRight");
        servo_platformLeft = hardwareMap.get(Servo.class, "servo_platformLeft");


        motor_left.setDirection(DcMotor.Direction.REVERSE);
        motor_right.setDirection(DcMotor.Direction.FORWARD);
        motor_center.setDirection(DcMotor.Direction.FORWARD);


        //Set up the motors and servos
        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo_grabber.resetDeviceConfigurationForOpMode();
        servo_gripRight.resetDeviceConfigurationForOpMode();
        servo_gripLeft.resetDeviceConfigurationForOpMode();

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


        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /*
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
        */

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.25f * mmPerInch;   // eg: Camera is 10.5 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 6.0f * mmPerInch;     // eg: Camera is 6.5 inches right on the robot

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        //Turn on flash
        //CameraDevice.getInstance().setFlashTorchMode(true);

        // Send telemetry message to indicate successful Encoder reset and initialization
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initialization Time:", ""+runtime.seconds()+"s");
        telemetry.addData("Starting Positions:",  "L:%7d R:%7d C:%7d",
                motor_left.getCurrentPosition(),
                motor_right.getCurrentPosition(),
                motor_center.getCurrentPosition());
        telemetry.addData("Center Motor Mode:", ""+motor_center.getMode());
        telemetry.update();


        //START OF PLAY
        waitForStart();
        runtime.reset();
        targetsSkyStone.activate();


        /*
         If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         */
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        //S T A R T   O F   D R I V E   C O D E
        OpenGLMatrix stoneLoc = null;

        //Drive the H-Drive Left 6 inches, except it doesn't work so it just goes left for 1 second instead
        //encoderDriveCenter(DRIVE_SPEED / 2.0, -6, 1.0, false);

        gyroDrive(0.4, 9.5, 0);

        boolean stoneDetected = false;
        double stoneX = -17.0;
        double stoneY = -7;
        double timeToDetect = runtime.seconds();
        String stoneConfig = "";
        while(!stoneDetected){
           getLocation();
            if(targetVisible){
                stoneDetected = true;
                telemetry.addData("STONE:", "DETECTED");
                VectorF translation = lastLocation.getTranslation();
                stoneX = translation.get(0) / mmPerInch;
                stoneY = translation.get(1)/ mmPerInch;
                encoderDriveCenterStoneCentering(0.2, 5.0);
            }
            else
                telemetry.addData("STONE:", "NOT DETECTED");
            telemetry.update();
            //Abort trying to detect a stone if it takes longer than 5 seconds;
            if(runtime.seconds() - timeToDetect > 5.0){
                break;
            }
        }
        if(stoneY < -6){
            stoneConfig = "Left";
        }
        else if(stoneY > 6){
            stoneConfig = "Right";
        }
        else{
            stoneConfig = "Middle";
        }


        stoneX = Math.round(stoneX * 10) / 10.0; //round off to one decimal place

        gyroDrive(DRIVE_SPEED, (-1*stoneX) - 2, 0);

        servo_grabber.setPosition(1.0);
        sleep(2000);

        telemetry.addData("Point: ", "1");
        gyroDrive(DRIVE_SPEED, -6.0, 0);
        telemetry.addData("Point: ", "2");

        servo_gripRight.setPosition(1.0);
        servo_gripLeft.setPosition(0.0);
        sleep(1000);

        gyroTurn(TURN_SPEED, 90.0);

        if(stoneConfig.equals("Left")){
            gyroDrive(DRIVE_SPEED, 78.0, 90);
        }
        else if(stoneConfig.equals("Middle")){
            gyroDrive(DRIVE_SPEED, 70.0, 90);
        }
        else{
            gyroDrive(DRIVE_SPEED, 62.0, 90);
        }

        telemetry.addData("Stone Config:", ""+stoneConfig);
        telemetry.update();
        sleep(10000);

        gyroTurn(TURN_SPEED, 0);

        gyroDrive(DRIVE_SPEED, 6.0, 0);

        servo_grabber.setPosition(0.0);
        sleep(1000);

        servo_gripRight.setPosition(0.0);
        servo_gripLeft.setPosition(1.0);
        sleep(1000);

        servo_grabber.setPosition(0.5);
        sleep(2000);

        /*
        OpenGLMatrix stoneLoc = getStoneLocation();
        double stoneX;
        double stoneY;
        boolean stoneDetected = false;
        while(!stoneDetected){
            stoneLoc = getStoneLocation();
        }
        if(stoneLoc != null){
            stoneX = stoneLoc.getTranslation().get(0); //X pos of stone
            stoneY = stoneLoc.getTranslation().get(1); //Y pos of stone
        }
        else{
            stoneX = 0;
            stoneY = 45;
        }

        if(stoneX > -4 && stoneX < 4){
            stoneConfig = "Middle";
        }
        else if(stoneX < -4){
            stoneConfig = "Right";
        }
        else if(stoneX > 4){
            stoneConfig = "Left";
        }

        if(stoneConfig.equals("Middle")){
            telemetry.addData("Stone Configuration:", "Middle");
            telemetry.update();
        }
        else if(stoneConfig.equals("Right")){
            telemetry.addData("Stone Configuration:", "Right");
            telemetry.update();
            gyroDrive(DRIVE_SPEED, 8.0, 0);
        }
        else if(stoneConfig.equals("Left")){
            telemetry.addData("Stone Configuration:", "Left");
            telemetry.update();
            gyroDrive(DRIVE_SPEED, -8.0, 0);
        }

         */


        /*
        //Drive forward To blocks and wait a second
        encoderDrive(DRIVE_SPEED,DRIVE_SPEED,  21,  21, 10.0, true);
        sleep(1000);

        //Turn right 90d so phone faces blocks
        encoderDrive(TURN_SPEED, TURN_SPEED, -13, 13, 10.0, false);

        //Scan block to see if it a Skystone, if not move forward 8 inches to next block, and repeat for each block
        xPos = -23;
        for(int i = 0; i < 5; i++){
            if(detectStone().equals("SKYSTONE"))
                break;
            sleep(250);
            encoderDrive(DRIVE_SPEED* 0.75,DRIVE_SPEED * 0.75,8,8, 10.0, false);
            xPos = -1*(31 + i*8); //Used to determine the xPos position of the robot based on which block it ends up on
        }

        //Turn left 90d so front faces block
        encoderDrive(TURN_SPEED, TURN_SPEED, 13, -13, 10.0, false);

        //Drive forward to reach block
        encoderDrive(DRIVE_SPEED,DRIVE_SPEED, 8,8, 10.0, true);

        //Lower the arm and wait a second
        servo_grabber.setPosition(1.0);
        sleep(2000);

        //Drive back with block
        encoderDrive(DRIVE_SPEED,DRIVE_SPEED, -8, -8, 10.0, true);

        //Grab the block and wait a second
        servo_gripRight.setPosition(1.0);
        servo_gripLeft.setPosition(0.0);
        sleep(2000);

        //Turn left 90d to face center
        encoderDrive(TURN_SPEED,TURN_SPEED, 13, -13, 10.0, false);

        //Drive to center line based on either last block picked or robot location
        if(targetVisible){
            xPos = lastLocation.getTranslation().get(0) / mmPerInch;
        }
        encoderDrive(DRIVE_SPEED,DRIVE_SPEED, -1* xPos, -1* xPos, 10.0, true); //Invert the xPos because robot is on negative side of field, NOT to drive backwards

        //Drive to center of foundation
        //encoderDrive(DRIVE_SPEED,DRIVE_SPEED, 48,48, 10.0, true);

        //Turn right to face foundation

         */


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


/*
USE THIS CODE TO FIND SKYSTONES
locationMatrix = getStoneLocation();
                int counter = 0;
                while(locationMatrix == null && counter < 10) {
                    counter++;
                    sleep(100);
                    locationMatrix = getStoneLocation();
                }
                if (locationMatrix == null) {
                    //Just go for the middle stone
                }
 */






    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motor_left.setPower(speed * Math.signum(distance));
            motor_right.setPower(speed * Math.signum(distance));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (Math.abs(motor_left.getCurrentPosition()) < newLeftTarget ) &&
                    (Math.abs(motor_right.getCurrentPosition()) < newRightTarget)) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer*P_DRIVE_COEFF;
                rightSpeed = speed + steer*P_DRIVE_COEFF;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motor_left.setPower(leftSpeed * Math.signum(distance));
                motor_right.setPower(rightSpeed* Math.signum(distance));

                // Display drive status for the driver.

                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      motor_left.getCurrentPosition(),
                        motor_right.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                getLocation();
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();

            }

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            telemetry.addData("Path Status:", "Motion Stopped");
            telemetry.update();

            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevLeftPos = Math.abs(motor_left.getCurrentPosition());
            int prevRightPos = Math.abs(motor_right.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_right.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(motor_left.getCurrentPosition()) - prevLeftPos != 0) ){
                prevLeftPos = Math.abs(motor_left.getCurrentPosition());
                prevRightPos = Math.abs(motor_right.getCurrentPosition());
                sleep(100);
                telemetry.addData("Path Status:", "Waiting for Rest");
                telemetry.update();
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition()); //Used to keep track of encoder positions
            motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Left:", " "+motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Right:", " "+motor_right.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroDriveLookForStone ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        boolean atDestination = false;
        OpenGLMatrix locationMatrix;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(distance) * DRIVE_COUNTS_PER_INCH);

            runtime.reset();
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motor_left.setPower(speed * Math.signum(distance));
            motor_right.setPower(speed * Math.signum(distance));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !atDestination) {
                locationMatrix = getStoneLocation();
                int counter = 0;
                while(locationMatrix == null && counter < 10) {
                    counter++;
                    sleep(100);
                    locationMatrix = getStoneLocation();
                }
                if (locationMatrix == null) {
                    //Just go for the middle stone
                }

                if(getStoneLocation() != null){

                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer*P_DRIVE_COEFF;
                rightSpeed = speed + steer*P_DRIVE_COEFF;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motor_left.setPower(leftSpeed * Math.signum(distance));
                motor_right.setPower(rightSpeed* Math.signum(distance));

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      motor_left.getCurrentPosition(),
                        motor_right.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);

            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevLeftPos = Math.abs(motor_left.getCurrentPosition());
            int prevRightPos = Math.abs(motor_right.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_right.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(motor_left.getCurrentPosition()) - prevLeftPos != 0) ){
                prevLeftPos = motor_left.getCurrentPosition();
                prevRightPos = motor_right.getCurrentPosition();
                sleep(100);
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition()); //Used to keep track of encoder positions
            motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Left:", " "+motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Right:", " "+motor_right.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            /**
             * CONVERT TO IMU
             */
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        motor_left.setPower(0);
        motor_right.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else if(Math.abs(error) < 10.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.4;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 20.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.6;
            leftSpeed = -rightSpeed;
        }
        else if(Math.abs(error) < 30.0){
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * (Math.signum(steer)) * 0.8;
            leftSpeed = -rightSpeed;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * Math.signum(steer);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motor_left.setPower(leftSpeed);
        motor_right.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double rSpeed, double lSpeed,
                             double rightInches, double leftInches,
                             double timeoutS, boolean rampDown){
        int newLeftTarget;
        int newRightTarget;
        double differenceCount = Integer.MAX_VALUE; //Set this to some positive value, doesn't matter what

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(Math.abs(leftInches) * DRIVE_COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(Math.abs(rightInches) * DRIVE_COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            motor_left.setPower(lSpeed * (leftInches/Math.abs(leftInches)));
            motor_right.setPower(rSpeed * (rightInches/Math.abs(rightInches)));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.abs(motor_left.getCurrentPosition()) < newLeftTarget ) &&
                    (Math.abs(motor_right.getCurrentPosition()) < newRightTarget)){

                double nlSpeed;
                double nrSpeed;
                double rem = ( (newLeftTarget - Math.abs(motor_left.getCurrentPosition())) + (newRightTarget - Math.abs(motor_right.getCurrentPosition())) ) / 2;

                if(rem < 50 && rampDown){
                    nlSpeed = rSpeed*0.5;
                    nrSpeed = lSpeed*0.5;
                }
                else {
                    nlSpeed = lSpeed;
                    nrSpeed = rSpeed;
                }
                motor_left.setPower(nlSpeed * (leftInches/Math.abs(leftInches)));
                motor_right.setPower(nrSpeed * (rightInches/Math.abs(rightInches)));

                // Display info about encoders and robot's position/heading for the driver.
                telemetry.addData("Target Position:",  "L:%7d R:%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Current Position",  "L:%7d R:%7d", motor_left.getCurrentPosition(), motor_right.getCurrentPosition());
                telemetry.addData("Remaining:", ""+rem);
                getLocation();
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
            telemetry.addData("Path Status:", "Finished driving"); telemetry.update();

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);
            telemetry.addData("Path Status:", "Motion stopped"); telemetry.update();


            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevLeftPos = Math.abs(motor_left.getCurrentPosition());
            int prevRightPos = Math.abs(motor_right.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_right.getCurrentPosition()) - prevRightPos != 0) && (Math.abs(motor_left.getCurrentPosition()) - prevLeftPos != 0) ){
                prevLeftPos = motor_left.getCurrentPosition();
                prevRightPos = motor_right.getCurrentPosition();
                sleep(100);
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition()); //Used to keep track of encoder positions
            motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_right.getCurrentPosition()) + Math.abs(motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Left:", " "+motor_left.getCurrentPosition());
                telemetry.addData("Encoder Reset Progress Right:", " "+motor_right.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void encoderDriveCenter(double speed, double inches, double timeoutS, boolean rampDown){
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = motor_center.getCurrentPosition() + (int)(Math.abs(inches) * CENTER_COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            motor_center.setPower(speed * Math.signum(inches));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.abs(motor_center.getCurrentPosition()) < newTarget )){

                double nSpeed;
                double rem = (newTarget - Math.abs(motor_center.getCurrentPosition()));

                if(rem < 50 && rampDown){
                    nSpeed = speed*0.5;
                }
                else {
                    nSpeed = speed;
                }
                motor_center.setPower(nSpeed * Math.signum(inches));

                // Display info about encoders and robot's position/heading for the driver.
                telemetry.addData("Target Position:",  "C:%7d", newTarget);
                telemetry.addData("Current Position",  "C:%7d", motor_center.getCurrentPosition());
                telemetry.addData("Remaining:", ""+rem);
                getLocation();
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
            telemetry.addData("Path Status:", "Finished driving"); telemetry.update();

            // Stop all motion;
            motor_center.setPower(0);
            telemetry.addData("Path Status:", "Motion stopped"); telemetry.update();


            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevPos = Math.abs(motor_center.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_center.getCurrentPosition()) - prevPos != 0) ){
                prevPos = motor_center.getCurrentPosition();
                sleep(100);
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_center.getCurrentPosition()); //Used to keep track of encoder positions
            motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_center.getCurrentPosition()) ;
                telemetry.addData("Encoder Reset Progress Center:", " "+motor_center.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDriveCenterStoneCentering(double speed, double timeoutS){
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //newTarget = motor_center.getCurrentPosition() + (int)(Math.abs(inches) * CENTER_COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            motor_center.setPower( speed * (Math.signum(lastLocation.getTranslation().get(1)) ) );

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((lastLocation.getTranslation().get(1) < -0.5) || (lastLocation.getTranslation().get(1) > 0.5 ))){

                motor_center.setPower( speed * (Math.signum(lastLocation.getTranslation().get(1)) ) );

                // Display info about encoders and robot's position/heading for the driver.
                telemetry.addData("Current Position",  "C:%7d", motor_center.getCurrentPosition());
                getLocation();
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
            telemetry.addData("Path Status:", "Finished driving"); telemetry.update();

            // Stop all motion;
            motor_center.setPower(0);
            telemetry.addData("Path Status:", "Motion stopped"); telemetry.update();


            //Checks to see if robot is still moving, and if it is, waits 100ms and checks again, and repeats. This makes sure it has settled
            int prevPos = Math.abs(motor_center.getCurrentPosition());
            sleep(100);
            while( (Math.abs(motor_center.getCurrentPosition()) - prevPos != 0) ){
                prevPos = motor_center.getCurrentPosition();
                sleep(100);
            }

            //Reset the encoders so they are zeroed
            double resetC = Math.abs(motor_center.getCurrentPosition()); //Used to keep track of encoder positions
            motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Path Status:", "Encoders stopped and reset started"); telemetry.update();
            //Let the reset occur
            double resetTime = runtime.seconds();
            while (Math.abs(resetC) > 0){
                resetC = Math.abs(motor_center.getCurrentPosition()) ;
                telemetry.addData("Encoder Reset Progress Center:", " "+motor_center.getCurrentPosition()); telemetry.update();
                //Just in case the robot gets stuck in this loop, it will try to reset again every second
                if(runtime.seconds() - resetTime > 1.0){
                    motor_center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetTime = runtime.seconds();
                }
                //idle();
            }

            motor_center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    public void positionDrive(double speed, double xPos, double yPos, double timeOutS){
        boolean targetReached = false;
        if(opModeIsActive()){
            while(!targetReached){
                telemetry.addData("Hello", "World");
            }
        }
    }

    private void getLocation(){
        //Vuforia tracking
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
        }
    }
    private OpenGLMatrix getStoneLocation(){
        //Vuforia tracking
        for (VuforiaTrackable trackable : allTrackables) {
            if (trackable.getName().equals("Stone Target") && ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    return robotLocationTransform;
                }
            }
        }
        return null;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.60;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }

    private String detectStone(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size()==1){
                for(Recognition recognition : updatedRecognitions){
                    if(recognition.getLabel().equals(LABEL_STONE))
                        return "STONE";
                    else if(recognition.getLabel().equals(LABEL_SKYSTONE))
                        return  "SKYSTONE";
                }
            }

        }
        return "NONE";
    }

}

//This is a possible better encoder
/*
    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) throws InterruptedException {

        double     COUNTS_PER_MOTOR_REV    = 560 ;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is the ratio between the motor axle and the wheel
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = (robot.flm.getCurrentPosition() + robot.rlm.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        newRightTarget = (robot.frm.getCurrentPosition() + robot.rrm.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ( (runtime.seconds() < timeoutS) &&
                (Math.abs(robot.flm.getCurrentPosition() + robot.rlm.getCurrentPosition()) /2 < newLeftTarget  &&
                        Math.abs(robot.frm.getCurrentPosition() + robot.rrm.getCurrentPosition())/2 < newRightTarget)) {
            double rem = (Math.abs(robot.flm.getCurrentPosition()) + Math.abs(robot.rlm.getCurrentPosition())+Math.abs(robot.frm.getCurrentPosition()) + Math.abs(robot.rrm.getCurrentPosition()))/4;
            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }
//Keep running until you are about two rotations out
            else if(rem > (1000) )
            {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if(rem > (200) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {
                NLspeed = Lspeed * (rem / 1000);
                NRspeed = Rspeed * (rem / 1000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            robot.flm.setPower(NLspeed);
            robot.rlm.setPower(NLspeed);
            robot.frm.setPower(NRspeed);
            robot.rrm.setPower(NRspeed);
        }
        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        robot.flm.setPower(0);
        robot.frm.setPower(0);
        robot.rlm.setPower(0);
        robot.rrm.setPower(0);
        // show the driver how close they got to the last target
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.addData("Path2",  "Running at %7d :%7d", robot.flm.getCurrentPosition(), robot.frm.getCurrentPosition());
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
        double resetC = ((Math.abs(robot.flm.getCurrentPosition()) + Math.abs(robot.rlm.getCurrentPosition())+ Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.frm.getCurrentPosition())));
        //Get the motor encoder resets in motion
        robot.flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rlm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rrm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            resetC =  ((Math.abs(robot.flm.getCurrentPosition()) + Math.abs(robot.rlm.getCurrentPosition())+ Math.abs(robot.frm.getCurrentPosition())+Math.abs(robot.frm.getCurrentPosition())));
            idle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        robot.flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rlm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//give the encoders a chance to switch modes.
        waitOneFullHardwareCycle();
        //  sleep(250);   // optional pause after each move
    }
*/
