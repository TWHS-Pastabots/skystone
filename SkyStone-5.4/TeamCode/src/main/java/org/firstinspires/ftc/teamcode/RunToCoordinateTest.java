package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@Autonomous
public class RunToCoordinateTest extends LinearOpMode  {

    private final double ANGLE_THRESHOLD = 3.0;
    private final double TURN_SPEED = 0.5;
    private final double DRIVE_SPEED = 0.5;
    private final int THREAD_SLEEP_DELAY = 50;
    private final double MINIMUM_POWER = 0.2;
    static final double     Kp  = 0.002;
    static final double     Ki  = 0.002;
    static final double     Kd  = 0.000;

    private final double START_X = 31.0;
    private final double START_Y = 8.0;
    private final double START_ORIENTATION = 90.0;

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();
    private ElapsedTime successTimer = new ElapsedTime();
    private ElapsedTime logTimer = new ElapsedTime();

    private TouchSensor touch;

    private OpenCvCamera phoneCam;
    private RunToCoordinateTest.DetectorPipeline detectorPipeline;
    private String stoneConfig;

    private double integral;

    private File actionLogInternal = new File("C:\\Users\\Matt\\Documents\\GitHub\\skystone\\SkyStone-5.4\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\actionLog.txt");
    private File actionLog = AppUtil.getInstance().getSettingsFile("actionLog.txt");
    String log;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        touch = hardwareMap.touchSensor.get("touch");

        ReadWriteFile.writeFile(actionLog, "Log Initiated" );

        robot.leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detectorPipeline = new DetectorPipeline();
        phoneCam.setPipeline(detectorPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT); //CHANGE BACK

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        Positioning positioning = new Positioning(robot.leftEnc, robot.rightEnc, robot.horizEnc, THREAD_SLEEP_DELAY, robot.imu, START_ORIENTATION, START_X, START_Y);
        Thread positionThread = new Thread(positioning);
        positionThread.start();

        SensorThread sensing = new SensorThread(positioning);
        Thread sensorThread = new Thread(sensing);
        sensorThread.start();

        telemetry.addData("Status", "Started Thread");
        telemetry.update();

        //START OF DRIVING

        runTime.reset();
        driveToPosition(20.0, 48.0, DRIVE_SPEED, 90, 0.0, 40.0, 10, positioning, sensing);
        sensing.activatePushBlock();
        robot.leftIn.setPower(-.7);
        robot.rightIn.setPower(-.7);
        driveToPosition(23.0, 48.0, DRIVE_SPEED, 90, 0.0, 12.0, 10, positioning, sensing);
        sleep(1000);
        robot.leftIn.setPower(0);
        robot.rightIn.setPower(0);
        driveToPosition( 23, 30.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        driveToPosition( -36, 30.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
        sensing.dropBlock();
        sleep(6000);
        driveToPosition( 44, 30.0, DRIVE_SPEED, 90, 0.0, 60.0, 10, positioning, sensing);
        driveToPosition( 44, 46.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        robot.leftIn.setPower(-.7);
        robot.rightIn.setPower(-.7);
        driveToPosition( 48, 46.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        sleep(1000);
        robot.leftIn.setPower(0);
        robot.rightIn.setPower(0);
        driveToPosition( 48, 30.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        driveToPosition( -36, 30.0, DRIVE_SPEED, 90, 0.0, 35.0, 10, positioning, sensing);
        sensing.dropBlock();
        sleep(6000);
        driveToPosition( 0, 30.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);

        telemetry.addData("Status:", "Finished Driving");
        telemetry.update();

        positioning.stop();
        sensing.stop();

    }

    public void driveToPosition(double targetX, double targetY, double speed, double heading, double rampUpTimeS, double rampDownDistance, double timeoutS, Positioning positioning, SensorThread sensing){
        double xDistance = targetX - positioning.getX();
        double yDistance = targetY - positioning.getY();
        double distance = Math.hypot(xDistance, yDistance);
        double movementAngle;

        double fLeft;
        double fRight;
        double rLeft;
        double rRight;
        double error;
        double steer;
        double previousError = 0;
        double previousDistance = distance;
        double velocity;
        boolean isRampingDown = false;
        int successCounter = 0;
        double  dt = 0;
        integral = 0;

        //Turn to the desired heading
        turn(heading, TURN_SPEED, positioning);

        ElapsedTime pidTimer = new ElapsedTime();
        moveTimer.reset();
        logTimer.reset();
        successTimer.reset();
        while(moveTimer.seconds() < timeoutS && distance > 1){
            xDistance = targetX - positioning.getX();
            yDistance = targetY - positioning.getY();
            distance = Math.hypot(xDistance, yDistance);
            movementAngle = Math.toDegrees(Math.atan2(xDistance, yDistance)) - positioning.getOrientation();
            velocity = (distance - previousDistance) / dt; //Calculated in inches/second

            error = getError(heading, positioning);
            pidTimer.reset();
            steer = getSteerPID(error, previousError, dt, Kp, Ki, Kd);
            previousError = error;

            //Convert the movement angle, which is relative to the y-axis to be relative to the x-axis for future calculations
            double angleXAxis = 90 - movementAngle;
            if(angleXAxis > 180)
                angleXAxis -= 360;
            else if(angleXAxis < -180)
                angleXAxis += 360;

            fLeft = 0;
            fRight = 0;
            rLeft = 0;
            rRight = 0;

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
                if(rampUpPercent < 0.5);
                    rampUpPercent = 0.5;
                fLeft = fLeft * (rampUpPercent);
                fRight = fRight * (rampUpPercent);
                rLeft = rLeft * (rampUpPercent);
                rRight = rRight * (rampUpPercent);
            }
            else if(distance < 4.0)
                isRampingDown = true;

            //Ramp down the motor powers
            if(isRampingDown){
                fLeft = fLeft * (0.5);
                fRight = fRight * (0.5);
                rLeft = rLeft * (0.5);
                rRight = rRight * (0.5);
            }

            //Provide steer to the motors based off the PID
            if(!isRampingDown) {
                fLeft += steer;
                rLeft += steer;
                fRight -= steer;
                rRight -= steer;
            }

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

            previousDistance = distance;

            //Display Global (x, y, theta) coordinates
            telemetry.addData("Lift Encoder Pos", robot.liftMotor.getCurrentPosition());
            telemetry.addData("X Position", positioning.getX());
            telemetry.addData("Y Position", positioning.getY());
            telemetry.addData("Orientation (Degrees)", positioning.getOrientation());
            telemetry.addData("Left  Front Motor P:", robot.leftFront.getPower());
            telemetry.addData("Right  Front Motor P:", robot.rightFront.getPower());
            telemetry.addData("Left  Rear Motor P:", robot.leftRear.getPower());
            telemetry.addData("Right Rear Motor P:", robot.rightRear.getPower());
            telemetry.update();
            if ( logTimer.milliseconds() > 100 ) {
                log += "Runtime::" + runTime.seconds() + "\n\t X Pos:: " + positioning.getX()
                        + " Y Pos:: " + positioning.getY() + " Orientation:: " + positioning.getOrientation() + " DtT:: +" + distance + "\n\t"
                        + "FL Motor Power:: " + robot.leftFront.getPower() + " BL Motor Power:: " + robot.leftRear.getPower() + " FR Motor Power:: " +
                        robot.rightFront.getPower() + " BR Motor Power:: " + robot.rightRear.getPower() + "\n";

                logTimer.reset();
            }

            //Set the time difference for the derivative
            dt = pidTimer.seconds();

        }

        log += "\n\n\n";

        //Stop the robot
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

        ReadWriteFile.writeFile(actionLog, log);
    }

    private void turn(double targetAngle, double turnSpeed, Positioning positioning){

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


    public double getError(double targetAngle, Positioning positioning) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - positioning.getOrientation();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteerPID(double error, double previousError, double dt, double Kp, double Ki, double Kd) {
        integral = integral + (error * dt);
        double derivative = (error - previousError) / dt;
        return Range.clip((error * Kp) + (Ki * integral) + (Kd * derivative), -1, 1);
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
        private final int cx0 = 75, cx1 = 140, cx2 = 220;// Width=320 Height=240
        private final int cy0 = 175, cy1 = 175, cy2 = 175;

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
                detectedPos = "Left";
            else if(sum1 < sum0 && sum1 < sum2)
                detectedPos = "Middle";
            else if(sum2 < sum1 && sum2 < sum0)
                detectedPos = "Right";


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
        private boolean pushBlock = false;
        private boolean pushBlockActivated = false;
        private boolean dropBlock = false;
        private boolean blockDropped = false;
        private Positioning positioning;
        private ElapsedTime liftTimer = new ElapsedTime();
        double pulleyCircumference = 2 * Math.PI * 1.0;
        double liftMotorGearRatio = 70.0 / 56.0;
        double liftEncoderCountsPerInch = (2240 * pulleyCircumference * liftMotorGearRatio) / 3.0;
        double targetLiftPosition = liftEncoderCountsPerInch * 4.0;
        double pos;

        public SensorThread(Positioning positioning){
            this.positioning = positioning;
        }

        public void stop(){
            isRunning = false;
        }

        public double getColorDistance(){
            return robot.blockInSensor.getDistance(DistanceUnit.INCH);
        }


        public void activatePushBlock(){
            pushBlockActivated = true;
        }

        public void dropBlock(){
            dropBlock = true;
        }

         @Override
         public void run(){
            while(isRunning){
                if(pushBlockActivated && robot.blockInSensor.getDistance(DistanceUnit.INCH) > 0.5 && !pushBlock)
                    pushBlock = true;
                if(pushBlock) {
                    robot.servo_blockPush.setPosition(0.0);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    while (positioning.getX() > 10)
                        try {
                            Thread.sleep(50);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    robot.claw.setPosition(1.0);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    liftTimer.reset();
                    while(liftTimer.seconds() < 1.5) {
                        robot.liftMotor.setPower(-0.5);
                    }
                    robot.liftMotor.setPower(0);
                    robot.servo_blockPush.setPosition(1.0);

                    while(!blockDropped){
                        if(dropBlock){
                            robot.clawT.setPosition(0.0);
                            try {
                                Thread.sleep(1000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            robot.claw.setPosition(0.0);
                            try {
                                Thread.sleep(500);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            robot.clawT.setPosition(1.0);
                            blockDropped = true;
                        }
                    }
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    while(!touch.isPressed()){
                        robot.liftMotor.setPower(0.5);
                    }
                    robot.liftMotor.setPower(0.0);
                    dropBlock = false;
                    pushBlock = false;
                    blockDropped = false;
                }
            }
         }
    }


}
