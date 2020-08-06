package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public abstract class PositionBasedAuton4 extends LinearOpMode {

    private final double ANGLE_THRESHOLD = 3.0;
    public final double TURN_SPEED = 0.7;
    public final double TURN_SPEED_HIGH = 1.0;
    public final double DRIVE_SPEED = 1.0;
    public final double DRIVE_SPEED_HIGH = 1.0;
    private final int THREAD_SLEEP_DELAY = 75; //25
    private final double MINIMUM_POWER = 0.2;

    private static final double hKp = 0.009;
    private static final double hKi = 0.003;
    private static final double hKd = 0.00;

    private static final double vKp = 0.0009;
    private static final double vKi = 0.0;
    private static final double vKd = 0.0;

    public double startX = 0.0;
    public double startY = 0.0;
    public double startOrientation = 0.0;
    public boolean isInPositiveX;
    public boolean isBlue = true;

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();
    private ElapsedTime successTimer = new ElapsedTime();
    private ElapsedTime logTimer = new ElapsedTime();
    private ElapsedTime dtDistTimer = new ElapsedTime();

    private double startingHeight;

    private TouchSensor touch;

    private OpenCvCamera phoneCam;
    private PositionBasedAuton4.DetectorPipeline detectorPipeline;
    private String stoneConfig;

    public Positioning2 positioning;
    public SensorThread sensing;
    public ExpansionHubReading ehr;
    public Logging logging;

    private double steerIntegral;
    private double velocityIntegral;

    private File actionLogInternal = new File("C:\\Users\\Matt\\Documents\\GitHub\\skystone\\SkyStone-5.4\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\actionLog.txt");
    private File actionLog = AppUtil.getInstance().getSettingsFile("actionLog.txt");
    public String log;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initializing");
        telemetry.addData("Currently:", "Setting Start Positions");
        telemetry.update();
        setStartPos();

        telemetry.addData("Status", "Initializing");
        telemetry.addData("Currently:", "Initializing Hardware Map");
        telemetry.update();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initializing");
        telemetry.addData("Currently:", "Setting Run Modes");
        telemetry.update();
        robot.leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initializing");
        telemetry.addData("Currently:", "Creating Expansion Hub List");
        telemetry.update();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Initializing");
        telemetry.addData("Currently:", "Initializing Camera");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detectorPipeline = new DetectorPipeline(isBlue);
        phoneCam.setPipeline(detectorPipeline);
        if(isBlue)
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        else
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("Status", "Initializing");
        telemetry.addData("Currently:", "Starting threads");
        telemetry.update();
        //Start Expansion Hub Reading Thread
        ehr = new ExpansionHubReading(robot, allHubs);
        Thread ehrThread = new Thread(ehr);
        ehrThread.start();

        //Start logging thread
        logging = new Logging();
        Thread loggingThread = new Thread(logging);
        loggingThread.start();
        logging.createLog("SensorLog");
        logging.createLog("ActionLog");
        logging.createLog("HeadingErrorLog");

        //Start positioning thread
        positioning = new Positioning2(ehr, THREAD_SLEEP_DELAY, startOrientation, startX, startY, logging);
        Thread positionThread = new Thread(positioning);
        positionThread.start();

        //Start sensor thread
        sensing = new SensorThread(positioning);
        Thread sensorThread = new Thread(sensing);
        sensorThread.start();

        //Init complete
        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();

        ehr.setReadHeight(true);
        sleep(100);
        while(!isStarted() && !isStopRequested()){
            startingHeight = ehr.armHeightD;
            stoneConfig = detectorPipeline.getDetectedPos();
            telemetry.addData("Stone Config: ", stoneConfig);
            telemetry.addData("Height ", ehr.armHeightD);
            telemetry.addData("Block ", ehr.blockD);
            telemetry.addData("Front ", ehr.frontD);
            telemetry.addData("Left ", ehr.leftD);
            telemetry.update();
        }

        runTime.reset();
        ehr.setReadHeight(false);
        drive();
        positioning.stop();
        sensing.stop();
        ehr.stop();
        logging.stop();
    }

    public String getStoneConfig(){
        return stoneConfig;
    }

    public void lowerHooks(){
        robot.leftH.setPosition(0.9);
        robot.rightH.setPosition(0.8);
    }

    public void raiseHooks(){
        robot.leftH.setPosition(0.0);
        robot.rightH.setPosition(0.0);
    }

    public abstract void setStartPos();

    public abstract void drive();

    public void correctPosition(boolean x, boolean y, Positioning2 positioning){
        ElapsedTime correctionTimer = new ElapsedTime();
        ehr.setReadCorrections(true);
        sleep(100);
        double front = ehr.frontD;
        double left = ehr.leftD;
        ehr.setReadCorrections(false);

        correctionTimer.reset();
        int counter = 0;
        while(correctionTimer.seconds() < 0.25 && opModeIsActive()) {
            front += ehr.frontD;
            left += ehr.leftD;
            counter++;
        }

        front /= counter;
        left /= counter;

        front *= Math.cos(Math.toRadians(90.0 - Math.abs(positioning.getOrientation())));
        left *= Math.cos(Math.toRadians(90.0 - Math.abs(positioning.getOrientation())));

        front += 7.0;
        front -= 70.0;
        left += 7.0;

        if(x) {
            positioning.correctX(front);
            logging.add("ActionLog", "\n\nX Correction: " + positioning.getX());
        }
        if(y) {
            positioning.correctY(left);
            logging.add("ActionLog", "\n\nY Correction: " + positioning.getY());
        }
    }


    public void driveToPosition(double targetX, double targetY, double velocity, double heading, double rampUpTimeS, double rampDownDistance, double timeoutS, Positioning2 positioning, SensorThread sensing){
        double xDistance = targetX - positioning.getX();
        double yDistance = targetY - positioning.getY();
        double distance = Math.hypot(xDistance, yDistance);
        double movementAngle;

        double fLeft;
        double fRight;
        double rLeft;
        double rRight;
        double speed; //Starting power for moves
        double headingError;
        double velocityError;
        double targetVelocity = velocity;
        double steer;
        double speedChange;
        double previousHeadingError = 0;
        double previousVelocityError = 0;
        boolean isRampingDown = false;
        boolean braked = false;
        double  dt = 0.0;

        double x;
        double y;
        double orientation;
        double vel;

        steerIntegral = 0;
        velocityIntegral = 0;

        ElapsedTime pidTimer = new ElapsedTime();
        moveTimer.reset();
        logTimer.reset();
        successTimer.reset();
        while(moveTimer.seconds() < timeoutS && distance > 2 && opModeIsActive()){
            /*
            positioning.setCorrectingX(false);
            positioning.setCorrectingY(false);
            if(ehr.frontD <= 45.0) {
                correctPosition(true, false, positioning);
                positioning.setCorrectingX(true);
            }
            if(ehr.leftD <= 45.0 && positioning.getX() < -14.0){
                correctPosition(false, true, positioning);
                positioning.setCorrectingY(true);
            }

             */

            x = positioning.getX();
            y = positioning.getY();
            orientation = positioning.getOrientation();
            vel = positioning.getVelocity();

            xDistance = targetX - x;
            yDistance = targetY - y;
            distance = Math.hypot(xDistance, yDistance);
            movementAngle = Math.toDegrees(Math.atan2(xDistance, yDistance)) - orientation;

            //Calc the steer using PID
            headingError = getHeadingError(heading, orientation);
            pidTimer.reset();
            steer = getSteerPID(headingError, previousHeadingError, dt, hKp, hKi, hKd);
            previousHeadingError = headingError;

            //Calc the velocity PID
            velocityError = targetVelocity - vel; //If error is positive, need to increae velocity
            speedChange = getVelocityPID(velocityError, previousVelocityError, dt, vKp, vKi, vKd );
            previousVelocityError = velocityError;

            //Convert the movement angle, which is relative to the y-axis to be relative to the x-axis for future calculations
            double angleXAxis = 90 - movementAngle;
            if(angleXAxis > 180)
                angleXAxis -= 360;
            else if(angleXAxis < -180)
                angleXAxis += 360;

            fLeft = 0.0;
            fRight = 0;
            rLeft = 0;
            rRight = 0;

            //speed = Range.clip(speed + ( ((int)(speedChange * 10000)) / 10000.0), 0.0, 1.0 );

            speed = velocity;
            if(distance < rampDownDistance && rampDownDistance > 0) {
                if (!braked) {
                    speed = 0.0;
                    braked = true;
                }
                else
                    speed = 0.3;
            }


            //Calculate the speeds based on the quadrant the robot is traveling to
            if(angleXAxis > 0 && angleXAxis < 90) {  //First Quadrant
                fLeft = speed;
                fRight = calcOtherPower(fLeft, 45.0, 135.0, angleXAxis);
                rRight = speed;
                rLeft = calcOtherPower(rRight, 45.0, 135.0, angleXAxis);
            }
            else if(angleXAxis > 90 && angleXAxis < 180){  //Second Quadrant
                fRight = speed;
                fLeft = calcOtherPower(fRight, 135.0, 45.0, angleXAxis);
                rLeft = speed;
                rRight = calcOtherPower(rLeft, 135.0, 45.0, angleXAxis);
            }
            else if(angleXAxis > -180 && angleXAxis < -90){ //Third Quadrant
                fLeft = -speed;
                fRight = calcOtherPower(fLeft, 45.0, 135.0, angleXAxis);
                rRight = -speed;
                rLeft = calcOtherPower(rRight, 45.0, 135.0, angleXAxis);
            }
            else if(angleXAxis < 0 && angleXAxis > -90){ //Fourth Quadrant
                fRight = -speed;
                fLeft = calcOtherPower(fRight, 135.0, 45.0, angleXAxis);
                rLeft = -speed;
                rRight = calcOtherPower(rLeft, 135.0, 45.0, angleXAxis);
            }
            else if( angleXAxis == 0){
                fLeft = speed;
                fRight = -speed;
                rLeft = -speed;
                rRight = speed;
            }
            else if(angleXAxis == 90.0){
                fLeft = speed;
                fRight = speed;
                rLeft = speed;
                rRight = speed;
            }
            else if( angleXAxis == 180.0 || angleXAxis == -180.0){
                fLeft = -speed;
                fRight = speed;
                rLeft = speed;
                rRight = -speed;
            }
            else if(angleXAxis == -90.0){
                fLeft = -speed;
                fRight = -speed;
                rLeft = -speed;
                rRight = -speed;
            }

            if(distance < rampDownDistance && rampDownDistance > 0)
                targetVelocity = 5.0;

            //Provide steer to the motors based off the PID
            fLeft += steer;
            rLeft += steer;
            fRight -= steer;
            rRight -= steer;


            //Make sure no motor is trying to go too fast
            double maxPower = Math.max(Math.max(fLeft, fRight), Math.max(rLeft, rRight));
            if(maxPower > 1){
                fLeft = fLeft / (maxPower);
                fRight = fRight / (maxPower);
                rLeft = rLeft / (maxPower);
                rRight = rRight / (maxPower);
            }


            //Give powers to the wheels
            robot.leftFront.setPower(fLeft);
            robot.rightFront.setPower(fRight);
            robot.leftRear.setPower(rLeft);
            robot.rightRear.setPower(rRight);


            //Display Global (x, y, theta) coordinates
            telemetry.addData("Velocity", vel);
            telemetry.addData("X Position", x);
            telemetry.addData("Y Position", y);
            telemetry.addData("Orientation (Degrees)", orientation);
            telemetry.addData("Speed Change:", speedChange);
            telemetry.addData("Speed:", speed);
            telemetry.update();
            logging.add("HeadingErrorLog", headingError + "\n" + moveTimer.seconds() + "\n");
            logging.add("ActionLog", "\n\nRuntime: " + runTime.seconds() + "\n\tX Position: " + x + "   Y Position: " + y + "   Orientation: " + orientation + "   Velocity: " + vel);
            //logging.add("SensorLog", "Runtime: " + runTime.seconds() + "\n\tLeft Distance: " + ehr.leftD + " Front Distance: " + ehr.frontD + " Arm Height: " + ehr.armHeightD + " Block Distance: " + ehr.blockD + "\n\n");
            /*
            if ( logTimer.milliseconds() > 50 ) {
                log += "Runtime::" + runTime.seconds() + "\n\t X Pos:: " + positioning.getX()
                        + " Y Pos:: " + positioning.getY() + " Orientation:: " + positioning.getOrientation() + " DtT:: +" + distance + "\n\t"
                        + "FL Motor Power:: " + ehr.leftFrontPow + " BL Motor Power:: " + ehr.leftRearPow + " FR Motor Power:: " +
                        ehr.rightFrontPow + " BR Motor Power:: " + ehr.rightRearPow +
                        "\n\t" + "Velocity:: " + positioning.getVelocity() +  " Target Velocity:: " + targetVelocity + " Speed Change:" +  speedChange + "\n\t"
                        + "Arm Height: " + ehr.armHeightD + "\n\n";

                logTimer.reset();
            }

             */

            //Set the time difference for the derivative
            dt = pidTimer.seconds();


        }

        logging.add("ActionLog", "\n\n\n");

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        sleep(250);

        log += "\n\n\n\n";

        ReadWriteFile.writeFile(actionLog, log);
    }

    public void turn(double targetAngle, double turnSpeed, Positioning2 positioning){

        double angleError;
        int successCounter = 0;

        successTimer.reset();
        while(successCounter < 5 && opModeIsActive()){

            angleError = targetAngle - positioning.getOrientation();
            double signedTurnSpeed = turnSpeed * Math.signum(angleError);

            //Ramp down
            if(Math.abs(angleError) < 20)
                signedTurnSpeed *= 0.25;
            else if(Math.abs(angleError) < 40)
                signedTurnSpeed *= 0.4;
            else if(Math.abs(angleError) < 60)
                signedTurnSpeed *= 0.5;
            else if(Math.abs(angleError) < 80)
                signedTurnSpeed *= 0.75;

            robot.leftFront.setPower(signedTurnSpeed);
            robot.rightFront.setPower(-signedTurnSpeed);
            robot.leftRear.setPower(signedTurnSpeed);
            robot.rightRear.setPower(-signedTurnSpeed);

            if(Math.abs(angleError) < ANGLE_THRESHOLD && successTimer.milliseconds() > 10) {
                successCounter++;
                successTimer.reset();
            }
            else if(Math.abs(angleError) > ANGLE_THRESHOLD)
                successCounter = 0;

            telemetry.addData("Successes:", successCounter);
            telemetry.addData("Error", Math.abs(angleError));
            telemetry.update();
        }

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
    }

    public void turnNoRampDown(double targetAngle, double turnSpeed, Positioning2 positioning){

        double angleError;
        int successCounter = 0;

        successTimer.reset();
        while(successCounter < 5 && opModeIsActive()){

            angleError = targetAngle - positioning.getOrientation();
            double signedTurnSpeed = turnSpeed * Math.signum(angleError);

            robot.leftFront.setPower(signedTurnSpeed);
            robot.rightFront.setPower(-signedTurnSpeed);
            robot.leftRear.setPower(signedTurnSpeed);
            robot.rightRear.setPower(-signedTurnSpeed);

            if(Math.abs(angleError) < ANGLE_THRESHOLD && successTimer.milliseconds() > 10) {
                successCounter++;
                successTimer.reset();
            }
            else if(Math.abs(angleError) > ANGLE_THRESHOLD)
                successCounter = 0;

            telemetry.addData("Successes:", successCounter);
            telemetry.addData("Error", Math.abs(angleError));
            telemetry.update();
        }

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
    }

    private double calcOtherPower(double firstPower, double firstAngleDegrees, double otherAngleDegrees, double finalAngleDegrees ){
        double th1 = Math.toRadians(firstAngleDegrees);
        double th2 = Math.toRadians(otherAngleDegrees);
        double thf = Math.toRadians(finalAngleDegrees);

        //Calculate the power using a formula that was derived using vector addition
        return ( (firstPower * Math.cos(th1) * Math.tan(thf)) - (firstPower * Math.sin(th1)) ) / ( Math.sin(th2) - (Math.cos(th2) * Math.tan(thf)) );
    }


    public double getHeadingError(double targetAngle, double orientation) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - orientation;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteerPID(double error, double previousError, double dt, double Kp, double Ki, double Kd) {
        steerIntegral = steerIntegral + (error * dt);
        double derivative = (error - previousError) / dt;
        return Range.clip((error * Kp) + (Ki * steerIntegral) + (Kd * derivative), -1, 1);
    }

    public double getVelocityPID(double error, double previousError, double dt, double Kp, double Ki, double Kd){
        velocityIntegral = velocityIntegral + (error * dt);
        double derivative = (error - previousError) / dt;
        return (error * Kp) + (Ki * velocityIntegral) + (Kd * derivative);
    }

    private void writeToFile (String log, File f)  throws IOException {
        FileWriter fr = new FileWriter(f);
        telemetry.addData("Final Log", ReadWriteFile.readFile(actionLog));
    }


    class DetectorPipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and rightEnc-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat mat0 = new Mat();
        private Mat mat1 = new Mat();
        private Mat mat2 = new Mat();

        private boolean madeMats = false;
        private boolean isBlue;

        private Mat mask0;
        private Mat mask1;
        private Mat mask2;


        private final Scalar BLACK = new Scalar(0,0,0);
        private final Scalar WHITE = new Scalar(256, 256, 256);
        private final int r = 10;
        private int cx0, cx1, cx2;// Width=320 Height=240
        private int cy0 , cy1, cy2;

        private String detectedPos;

        public DetectorPipeline(boolean isBlue){
            if(isBlue){
                cx0 = 40;
                cx1 = 130;
                cx2 = 210;
                cy0 = 75;
                cy1 = 75;
                cy2 = 75;
            }
            else{
                cx0 = 30;
                cx1 = 120;
                cx2 = 200;
                cy0 = 200;
                cy1 = 200;
                cy2 = 200;
            }
            this.isBlue = isBlue;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            int h = input.height();
            int w = input.width();
            int t = input.type();
            if(!madeMats){
                mask0 = new Mat(h,w,t);
                mask1 = new Mat(h,w,t);
                mask2 = new Mat(h,w,t);
                madeMats = true;
            }

            mask0.setTo(BLACK);
            mask1.setTo(BLACK);
            mask2.setTo(BLACK);

            Imgproc.circle(mask0, new Point(cx0, cy0), r, WHITE, Core.FILLED);
            Imgproc.circle(mask1, new Point(cx1, cy1), r, WHITE, Core.FILLED);
            Imgproc.circle(mask2, new Point(cx2, cy2), r, WHITE, Core.FILLED);

            Core.bitwise_and(mask0, input, mat0);
            Core.bitwise_and(mask1, input, mat1);
            Core.bitwise_and(mask2, input, mat2);


            double sum0 = sum(Core.sumElems(mat0).val);
            double sum1 = sum(Core.sumElems(mat1).val);
            double sum2 = sum(Core.sumElems(mat2).val);

            if(isBlue){
                if(sum0 < sum1 && sum0 < sum2)
                    detectedPos = "Right";
                else if(sum1 < sum0 && sum1 < sum2)
                    detectedPos = "Middle";
                else if(sum2 < sum1 && sum2 < sum0)
                    detectedPos = "Left";
            }
            else{
                if(sum0 < sum1 && sum0 < sum2)
                    detectedPos = "Left";
                else if(sum1 < sum0 && sum1 < sum2)
                    detectedPos = "Middle";
                else if(sum2 < sum1 && sum2 < sum0)
                    detectedPos = "Right";
            }

            /*
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            Imgproc.circle(input, new Point(cx0, cy0), r, WHITE, Core.FILLED);
            Imgproc.circle(input, new Point(cx1, cy1), r, WHITE, Core.FILLED);
            Imgproc.circle(input, new Point(cx2, cy2), r, WHITE, Core.FILLED);

            return input;
        }

        public double sum(double[] arr){
            double sum = 0.0;
            for(int i = 0; i < arr.length; i++){
                sum += arr[i];
            }
            return sum;
        }

        public String getDetectedPos(){
            return  detectedPos;
        }
    }

    public class SensorThread implements Runnable{

        private boolean isRunning = true;
        private boolean dropBlockRequested = false;
        private boolean intakeActivated = false;
        private boolean blockDropped = false;
        private boolean axisFlipped = false;
        private boolean armRaised = false;
        private Positioning2 positioning;
        private ElapsedTime liftTimer = new ElapsedTime();
        double pulleyCircumference = 2 * Math.PI * 1.0;
        double liftMotorGearRatio = 70.0 / 56.0;
        double liftEncoderCountsPerInch = (2240 * pulleyCircumference * liftMotorGearRatio) / 3.0;
        double targetLiftPosition = 1000; //liftEncoderCountsPerInch * 4.0
        double xPos;
        private ElapsedTime loweringTimer = new ElapsedTime();
        private ElapsedTime intakeTimer = new ElapsedTime();

        public SensorThread(Positioning2 positioning){
            this.positioning = positioning;
        }

        public boolean isBlockDropped(){
            return blockDropped;
        }

        public double getEncPos(){
            return robot.liftMotor.getCurrentPosition();
        }

        public void stop(){
            isRunning = false;
        }

        public String activateIntake(){
            intakeActivated = true;
            return "worked";
        }

        public void deActivateIntake(){
            intakeActivated = false;
        }

        public void dropBlock(){
            dropBlockRequested = true;
        }

        public void flipAxis(){axisFlipped = true;}

        public void raiseArm(){

        }

        private void closeClaw(){
            if(isRunning)robot.claw.setPosition(1.0);
        }

        private void openClaw(){
            if(isRunning)robot.claw.setPosition(0.0);
        }

        private void turnClawIn(){
            if(isRunning)robot.clawT.setPosition(1.0);
        }

        private void turnClawOut(){
            if(isRunning)robot.clawT.setPosition(0.0);
        }

        private void activateSpanker(){
            if(isRunning)robot.servo_blockPush.setPosition(0.0);
        }

        private void retractSpanker(){
            if(isRunning)robot.servo_blockPush.setPosition(1.0);
        }

        private void turnOnIntake(){
            if(isRunning) {
                robot.leftIn.setPower(-1.0);
                robot.rightIn.setPower(-1.0);
            }
        }

        private void turnOffIntake(){
            if(isRunning) {
                robot.leftIn.setPower(0.0);
                robot.rightIn.setPower(0.0);
            }
        }

        private void sleep(int timeMs){
            try {
                Thread.sleep(timeMs);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

         @Override
         public void run(){
            while(isRunning){

                if(intakeActivated ){
                    //Turn on the intake and keep it on until a block is inside the robot
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    turnOnIntake();
                    intakeTimer.reset();
                    while(intakeTimer.seconds() < 2.0 && isRunning){
                        turnOnIntake();
                    }
                    activateSpanker();
                    sleep(500);
                    turnOffIntake();
                    closeClaw();
                    sleep(1000);

                    //Push the block into place and lower claw onto it only partially
                    //sleep(1000);


                    //Wait until the robot has crossed into positive x
                    /*
                    while (positioning.getX() > 0)
                        telemetry.addData("Status:", "Waiting to cross axis");

                     */


                    //Wait to cross the x-axis
                    /*
                    xPos = positioning.getX();
                    while(axisFlipped ? xPos < 0 : xPos > 0 && isRunning){
                        xPos = positioning.getX();
                    }
                    //Grab the block with the claw and raise the lift into position then turn the claw out
                    robot.liftMotor.setPower(-0.75);
                    ehr.setReadHeight(true);
                    sleep(100);
                    while(ehr.armHeightD < startingHeight + 9.0 && isRunning) {
                        xPos = positioning.getX();
                    }
                    ehr.setReadHeight(false);
                    robot.liftMotor.setPower(0);

                     */

                    retractSpanker();

                    //Wait until dropping the block is requested, then let go of the block and turn the claw back in
                    while(!blockDropped  && isRunning ){
                        if(dropBlockRequested){
                            robot.liftMotor.setPower(-0.75);
                            ehr.setReadHeight(true);
                            sleep(100);
                            while(ehr.armHeightD < startingHeight + 9.0 && isRunning) {
                                xPos = positioning.getX();
                            }
                            robot.liftMotor.setPower(0);
                            turnClawOut();
                            sleep(500);
                            robot.liftMotor.setPower(0.3);
                            loweringTimer.reset();
                            while(ehr.armHeightD > startingHeight + 4.0  && loweringTimer.seconds() < 1.0 && isRunning) {
                                robot.liftMotor.setPower(0.3);
                            }
                            robot.liftMotor.setPower(0.0);
                            openClaw();
                            robot.liftMotor.setPower(-0.75);
                            while(ehr.armHeightD < startingHeight + 7.0  && isRunning) {
                                robot.liftMotor.setPower(-0.75);
                            }
                            robot.liftMotor.setPower(0);
                            closeClaw();
                            turnClawIn();
                            sleep(750);
                            while(ehr.armHeightD > startingHeight + 1.0 && isRunning){
                                robot.liftMotor.setPower(0.5);
                            }
                            ehr.setReadHeight(false);
                            robot.liftMotor.setPower(0.0);
                            blockDropped = true;
                        }
                    }
                    openClaw();

                    //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    //Reset all the conditions to false
                    dropBlockRequested = false;
                    blockDropped = false;
                    intakeActivated = false;
                }

            }
         }
    }


}
