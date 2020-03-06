package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

public abstract class PositionBasedAuton3 extends LinearOpMode {

    private final double ANGLE_THRESHOLD = 3.0;
    public final double TURN_SPEED = 0.7;
    public final double TURN_SPEED_HIGH = 1.0;
    public final double DRIVE_SPEED = 1.0;
    public final double DRIVE_SPEED_HIGH = 1.0;
    private final int THREAD_SLEEP_DELAY = 50;
    private final double MINIMUM_POWER = 0.2;

    private static final double hKp = 0.005;
    private static final double hKi = 0.002;
    private static final double hKd = 0.000;

    private static final double vKp = 0.001;
    private static final double vKi = 0.001;
    private static final double vKd = 0.001;

    public double startX = 0.0;
    public double startY = 0.0;
    public double startOrientation = 0.0;
    public boolean isInPositiveX;

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();
    private ElapsedTime successTimer = new ElapsedTime();
    private ElapsedTime logTimer = new ElapsedTime();
    private ElapsedTime dtDistTimer = new ElapsedTime();
    private DistanceSensor armHeightDistance;

    private TouchSensor touch;

    private OpenCvCamera phoneCam;
    private PositionBasedAuton3.DetectorPipeline detectorPipeline;
    private String stoneConfig;

    public Positioning positioning;
    public SensorThread sensing;

    private double steerIntegral;
    private double velocityIntegral;

    private File actionLogInternal = new File("C:\\Users\\Matt\\Documents\\GitHub\\skystone\\SkyStone-5.4\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\actionLog.txt");
    private File actionLog = AppUtil.getInstance().getSettingsFile("actionLog.txt");
    public String log;

    @Override
    public void runOpMode(){
        setStartPos();
        robot.init(hardwareMap);
        touch = hardwareMap.touchSensor.get("touch");

        ReadWriteFile.writeFile(actionLog, "Log Initiated" );

        robot.leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armHeightDistance = hardwareMap.get(DistanceSensor.class, "armHeightDistance");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detectorPipeline = new DetectorPipeline();
        phoneCam.setPipeline(detectorPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT); //CHANGE BACK

        //Start positioning thread
        positioning = new Positioning(robot.leftEnc, robot.rightEnc, robot.horizEnc, THREAD_SLEEP_DELAY, robot.imu, startOrientation, startX, startY);
        Thread positionThread = new Thread(positioning);
        positionThread.start();

        //Start sensor thread
        sensing = new SensorThread(positioning);
        Thread sensorThread = new Thread(sensing);
        sensorThread.start();

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        //Find the current configuration of the skystones
        while(!isStarted()) {
            stoneConfig = detectorPipeline.getDetectedPos();
            telemetry.addData("Stone Config: ", stoneConfig);
            telemetry.addData("Height ",armHeightDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        waitForStart();
        runTime.reset();
        drive();
        positioning.stop();
        sensing.stop();

    }

    public String getStoneConfig(){
        return stoneConfig;
    }

    public void lowerHooks(){
        robot.leftH.setPosition(0);
        robot.rightH.setPosition(1);
    }

    public void raiseHooks(){
        robot.leftH.setPosition(1);
        robot.rightH.setPosition(0);
    }

    public abstract void setStartPos();

    public abstract void drive();


    public void driveToPosition(double targetX, double targetY, double velocity, double heading, double rampUpTimeS, boolean rampDown, double timeoutS, Positioning positioning, SensorThread sensing){
        double xDistance = targetX - positioning.getX();
        double yDistance = targetY - positioning.getY();
        double distance = Math.hypot(xDistance, yDistance);
        double startDistance = distance;
        double movementAngle;

        double fLeft;
        double fRight;
        double rLeft;
        double rRight;
        double speed = 0;
        double headingError;
        double velocityError;
        double targetVelocity = velocity;
        double startTargetVelocity = velocity;
        double steer;
        double speedChange;
        double previousHeadingError = 0;
        double previousVelocityError = 0;
        double previousDistance = distance;
        boolean isRampingDown = false;
        int successCounter = 0;
        double  dt = 0.0;
        double  dtDist = 0.0;
        steerIntegral = 0;
        velocityIntegral = 0;

        //Turn to the desired heading
        //turn(heading, TURN_SPEED, positioning);

        ElapsedTime pidTimer = new ElapsedTime();
        moveTimer.reset();
        logTimer.reset();
        successTimer.reset();
        while(moveTimer.seconds() < timeoutS && distance > 1){
            isInPositiveX = positioning.isInPositiveX();
            xDistance = targetX - positioning.getX();
            yDistance = targetY - positioning.getY();
            if(dtDistTimer.seconds() > 0.05){
                dtDist = dtDistTimer.seconds();
                previousDistance = distance;
                dtDistTimer.reset();
            }
            distance = Math.hypot(xDistance, yDistance);
            movementAngle = Math.toDegrees(Math.atan2(xDistance, yDistance)) - positioning.getOrientation();

            //Calc the steer using PID
            headingError = getHeadingError(heading, positioning);
            pidTimer.reset();
            steer = getSteerPID(headingError, previousHeadingError, dt, hKp, hKi, hKd);
            previousHeadingError = headingError;

            //Calc the velocity PID
            velocityError = targetVelocity - positioning.getVelocity(); //If error is positive, need to increae velocity
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

            speed = Range.clip(speed + ( ((int)(speedChange * 10000)) / 10000.0), 0.0, 1.0 );

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


            //Ramp up the motor powers
            if(rampUpTimeS > moveTimer.seconds()) {
                double rampUpPercent = moveTimer.seconds() / rampUpTimeS;
                if(rampUpPercent < 0.5)
                    rampUpPercent = 0.5;
                fLeft = fLeft * (rampUpPercent);
                fRight = fRight * (rampUpPercent);
                rLeft = rLeft * (rampUpPercent);
                rRight = rRight * (rampUpPercent);
            }
            else if(distance < (startDistance / 2.0) && rampDown)
                targetVelocity = velocity / 2.0;
            else if(distance < (startDistance / 3.0) && rampDown)
                targetVelocity = velocity / 3.0;
            else if(distance < (startDistance / 4.0) && rampDown)
                targetVelocity = velocity / 4.0;
            else if(distance < (5) && rampDown)
                targetVelocity = 5.0;

            //Provide steer to the motors based off the PID
            fLeft += steer;
            rLeft += steer;
            fRight -= steer;
            rRight -= steer;

            //Make sure there is a minimum power being provided to the motors during a ramp down or up
            /*
            double minPower = Math.min(Math.min(fLeft, fRight), Math.min(rLeft, rRight));
            if(minPower < MINIMUM_POWER && (isRampingDown || rampUpTimeS > moveTimer.seconds()) ){
                double powerMultiplier = MINIMUM_POWER / minPower;
                fLeft = fLeft * (powerMultiplier);
                fRight = fRight * (powerMultiplier);
                rLeft = rLeft * (powerMultiplier);
                rRight = rRight * (powerMultiplier);
            }

             */

            //Make sure no motor is trying to go too fast
            double maxPower = Math.max(Math.max(fLeft, fRight), Math.max(rLeft, rRight));
            if(maxPower > 1){
                fLeft = fLeft / (maxPower);
                fRight = fRight / (maxPower);
                rLeft = rLeft / (maxPower);
                rRight = rRight / (maxPower);
            }

            //Ramp down the motor powers
            /*
            if(distance < rampDownDistance){
                double rampDownPercent = distance / rampDownDistance;
                if(rampDownPercent < 0.3)
                    rampDownPercent = 0.3;
                fLeft = fLeft * (rampDownPercent);
                fRight = fRight * (rampDownPercent);
                rLeft = rLeft * (rampDownPercent);
                rRight = rRight * (rampDownPercent);
            }

             */

            //Give powers to the wheels
            robot.leftFront.setPower(fLeft);
            robot.rightFront.setPower(fRight);
            robot.leftRear.setPower(rLeft);
            robot.rightRear.setPower(rRight);

            //Check if robot is in the correct location
            /*
            if(distance < 1 && successTimer.milliseconds() > 20){
                successCounter++;
                successTimer.reset();
            }
            else if(distance > 1)
                successCounter = 0;

             */



            //Display Global (x, y, theta) coordinates
            telemetry.addData("Lift Encoder Pos", robot.liftMotor.getCurrentPosition());
            telemetry.addData("X Position", positioning.getX());
            telemetry.addData("Y Position", positioning.getY());
            telemetry.addData("Orientation (Degrees)", positioning.getOrientation());
            telemetry.addData("Velocity:", positioning.getVelocity());
            telemetry.addData("Velocity Error:", velocityError);
            telemetry.addData("Speed Change:", speedChange);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Left  Front Motor P:", robot.leftFront.getPower());
            telemetry.addData("Right  Front Motor P:", robot.rightFront.getPower());
            telemetry.addData("Left  Rear Motor P:", robot.leftRear.getPower());
            telemetry.addData("Right Rear Motor P:", robot.rightRear.getPower());
            telemetry.addData("Left  Intake Motor P:", robot.leftIn.getPower());
            telemetry.addData("Right Intake Motor P:", robot.rightIn.getPower());
            telemetry.update();
            if ( logTimer.milliseconds() > 50 ) {
                log += "Runtime::" + runTime.seconds() + "\n\t X Pos:: " + positioning.getX()
                        + " Y Pos:: " + positioning.getY() + " Orientation:: " + positioning.getOrientation() + " DtT:: +" + distance + "\n\t"
                        + "FL Motor Power:: " + robot.leftFront.getPower() + " BL Motor Power:: " + robot.leftRear.getPower() + " FR Motor Power:: " +
                        robot.rightFront.getPower() + " BR Motor Power:: " + robot.rightRear.getPower() + "\n\t" + "Velocity:: " + positioning.getVelocity() +  " Previous Distance:: "
                        + previousDistance + " Distance:: " + distance + " dt:: " + dtDist + "\n"   ;

                logTimer.reset();
            }

            //Set the time difference for the derivative
            dt = pidTimer.seconds();


        }

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

        log += "\n\n\n";

        ReadWriteFile.writeFile(actionLog, log);
    }

    public void turn(double targetAngle, double turnSpeed, Positioning positioning){

        double angleError;
        int successCounter = 0;

        successTimer.reset();
        while(successCounter < 5){

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

    private double calcOtherPower(double firstPower, double firstAngleDegrees, double otherAngleDegrees, double finalAngleDegrees ){
        double th1 = Math.toRadians(firstAngleDegrees);
        double th2 = Math.toRadians(otherAngleDegrees);
        double thf = Math.toRadians(finalAngleDegrees);

        //Calculate the power using a formula that was derived using vector addition
        return ( (firstPower * Math.cos(th1) * Math.tan(thf)) - (firstPower * Math.sin(th1)) ) / ( Math.sin(th2) - (Math.cos(th2) * Math.tan(thf)) );
    }


    public double getHeadingError(double targetAngle, Positioning positioning) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - positioning.getOrientation();
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
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat mat0 = new Mat();
        private Mat mat1 = new Mat();
        private Mat mat2 = new Mat();

        private boolean madeMats = false;

        private Mat mask0;
        private Mat mask1;
        private Mat mask2;


        private final Scalar BLACK = new Scalar(0,0,0);
        private final Scalar WHITE = new Scalar(256, 256, 256);
        private final int r = 10;
        private final int cx0 = 40, cx1 = 130, cx2 = 210;// Width=320 Height=240
        private final int cy0 = 75, cy1 = 75, cy2 = 75;

        private String detectedPos;

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

            if(sum0 < sum1 && sum0 < sum2)
                detectedPos = "Right";
            else if(sum1 < sum0 && sum1 < sum2)
                detectedPos = "Middle";
            else if(sum2 < sum1 && sum2 < sum0)
                detectedPos = "Left";


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
        private Positioning positioning;
        private ElapsedTime liftTimer = new ElapsedTime();
        double pulleyCircumference = 2 * Math.PI * 1.0;
        double liftMotorGearRatio = 70.0 / 56.0;
        double liftEncoderCountsPerInch = (2240 * pulleyCircumference * liftMotorGearRatio) / 3.0;
        double targetLiftPosition = 1000; //liftEncoderCountsPerInch * 4.0
        double xPos;

        public SensorThread(Positioning positioning){
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

        private void closeClaw(){
            robot.claw.setPosition(1.0);
        }

        private void openClaw(){
            robot.claw.setPosition(0.0);
        }

        private void lowerClawPartially(){
            robot.claw.setPosition(0.65);
        }

        private void turnClawIn(){
            robot.clawT.setPosition(1.0);
        }

        private void turnClawOut(){
            robot.clawT.setPosition(0.0);
        }

        private void activateSpanker(){
            robot.servo_blockPush.setPosition(0.0);
        }

        private void retractSpanker(){
            robot.servo_blockPush.setPosition(1.0);
        }

        private void turnOnIntake(){
            robot.leftIn.setPower(-1.0);
            robot.rightIn.setPower(-1.0);
        }

        private void turnOffIntake(){
            robot.leftIn.setPower(0.0);
            robot.rightIn.setPower(0.0);
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
                    while(intakeActivated ){
                        turnOnIntake();
                    }
                    turnOffIntake();

                    //Push the block into place and lower claw onto it only partially
                    activateSpanker();
                    sleep(250);
                    closeClaw();

                    //Wait until the robot has crossed into positive x
                    /*
                    while (positioning.getX() > 0)
                        telemetry.addData("Status:", "Waiting to cross axis");

                     */


                    xPos = positioning.getX();
                    //Grab the block with the claw and raise the lift into position then turn the claw out
                    while(armHeightDistance.getDistance(DistanceUnit.INCH) < 14 && axisFlipped ? xPos > 0 : xPos < 0) {
                        robot.liftMotor.setPower(-0.75);
                        xPos = positioning.getX();
                    }
                    robot.liftMotor.setPower(0);
                    turnClawOut();
                    retractSpanker();

                    //Wait until dropping the block is requested, then let go of the block and turn the claw back in
                    while(!blockDropped && opModeIsActive()){
                        if(dropBlockRequested && opModeIsActive()){
                            openClaw();
                            sleep(500);
                            closeClaw();
                            turnClawIn();
                            blockDropped = true;
                        }
                    }
                    sleep(750);

                    //Lower the lift back down and open the claw
                    while(armHeightDistance.getDistance(DistanceUnit.INCH) > 2.0){
                        robot.liftMotor.setPower(0.15);
                    }
                    robot.liftMotor.setPower(0.0);
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
