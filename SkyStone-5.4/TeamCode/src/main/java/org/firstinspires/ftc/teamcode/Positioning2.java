package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.reference_code.PositionBasedAuton;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Positioning2 implements Runnable {

    //The amount of encoder ticks for each inch the robot moves.
    private final double COUNTS_PER_INCH = 1084.85144; //1141.9488791276

    //Expansion Hub Reading Thread
    private ExpansionHubReading ehr;

    //Tick offsets for each encoder for turning
    private double verticalLeftEncoderTickOffsetPerDegree;
    private double verticalRightEncoderTickOffsetPerDegree;
    private double horizontalEncoderTickOffsetPerDegree;

    //Positions of the encoders
    private double previousRightEncoderPosition = 0, previousLeftEncoderPosition = 0, previousHorizEncoderPosition = 0;
    private double verticalRightEncoderPosition = 0, verticalLeftEncoderPosition = 0, horizontalEncoderPosition = 0;

    //Values for calculating orientation
    private double orientation;
    private double previousOrientation;
    private double rawOrientationPrevious = 0.0;

    //The coordinates of the robot on the field, in inches
    private double xPos;
    private double yPos;
    private double xPosPrev;
    private double yPosPrev;
    private double prevTime = 0.0;

    private int readingID = 0;

    //The current velocity of the robot, in inches per second
    private double velocity;
    private ElapsedTime vTimer = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();

    //Files used for retrieving the tick offsets
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private File verticalLeftTickOffestFile = AppUtil.getInstance().getSettingsFile("verticalLeftTickOffset.txt");
    private File verticalRightTickOffestFile = AppUtil.getInstance().getSettingsFile("verticalRightTickOffset.txt");

    //Thread run condition
    private boolean isRunning = true;
    private boolean correctingX = false;
    private boolean correctingY = false;

    private ElapsedTime velocityTimer = new ElapsedTime();

    //Interval between each run of the thread
    private int sleepTime;

    private Logging log;

    public Positioning2(ExpansionHubReading ehr, int threadSleepDelay, double startOrientation, double startX, double startY, Logging log){
        this.ehr = ehr;
        sleepTime = threadSleepDelay;
        this.orientation = startOrientation;
        this.previousOrientation = startOrientation;
        this.xPos = startX;
        this.yPos = startY;
        this.xPosPrev = startX;
        this.yPosPrev = startY;
        this.log = log;
        log.createLog("PositionLog");
        log.createLog("YPosLog");
        log.createLog("VelocityLog");


        this.verticalLeftEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(verticalLeftTickOffestFile).trim());
        this.verticalRightEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(verticalRightTickOffestFile).trim());
        this.horizontalEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    //Updates the x and y positions of the robot
    private void updatePosition () {
        if(ehr.getReadingID() != readingID){
            readingID = ehr.getReadingID();
            //Find the current values for the encoders and orientation
            updateOrientation();
            updateVelocity();

            verticalLeftEncoderPosition = ehr.leftEnc;
            verticalRightEncoderPosition = ehr.rightEnc;
            horizontalEncoderPosition = ehr.horizEnc;

            //Calculate the changes in each encoder as well as the orientation
            double orientationChange = orientation - previousOrientation;
            double leftChange = (verticalLeftEncoderPosition - previousLeftEncoderPosition) - (orientationChange * verticalLeftEncoderTickOffsetPerDegree);
            double rightChange = (verticalRightEncoderPosition - previousRightEncoderPosition) - (orientationChange * verticalRightEncoderTickOffsetPerDegree);
            double horizontalChange = (horizontalEncoderPosition - previousHorizEncoderPosition) - (orientationChange * horizontalEncoderTickOffsetPerDegree);
            double h = horizontalChange;
            double v = (leftChange + rightChange) / 2.0;
            double orientationRadians = Math.toRadians(orientation);

            //Calculate and update the position
            //xPosPrev = xPos;
            xPos += (v * Math.sin(orientationRadians) + h * Math.cos(orientationRadians)) / COUNTS_PER_INCH;
            //yPosPrev = yPos;
            yPos += (v*Math.cos(orientationRadians) - h*Math.sin(orientationRadians)) / COUNTS_PER_INCH;

            //Set up each of the previous values to be ready for the next iteration
            previousLeftEncoderPosition = verticalLeftEncoderPosition;
            previousRightEncoderPosition = verticalRightEncoderPosition;
            previousHorizEncoderPosition = horizontalEncoderPosition;
            previousOrientation = orientation;
            log.add("PositionLog", "\nX Position: " + xPos + "   Y Position: " + yPos + "   Orientation: " + orientation +  "   Velocity: " + velocity);
            log.add("YPosLog", yPos + "\n" + elapsedTime.seconds() + "\n");
            log.add("VelocityLog", velocity + "\n" + elapsedTime.seconds() + "\n");
        }

    }

    //Updates the orientation of the robot based on the change in the raw imu values and the given initial orientation
    private void updateOrientation(){
        double rawOrientation = getAngle();
        double rawOrientationChange = rawOrientation - rawOrientationPrevious;

        if(rawOrientationChange < -180)
            rawOrientationChange += 360;
        else if(rawOrientationChange > 180)
            rawOrientationChange -=360;

        rawOrientationPrevious = rawOrientation;
        orientation += rawOrientationChange;
    }

    private void updateVelocity(){
        double deltaX = xPos - xPosPrev;
        double deltaY = yPos - yPosPrev;
        xPosPrev = xPos;
        yPosPrev = yPos;
        double dt = elapsedTime.seconds() - prevTime;
        prevTime = elapsedTime.seconds();

        //Calculate the velocity as the overall change in distance over the time the move took
        velocity = Math.hypot(deltaX, deltaY) / dt;
        log.add("PositionLog", "\n\nDeltaX: " + deltaX + "   DeltaY: " + deltaY + "   Hypot: " + Math.hypot(deltaX, deltaY) + "   dt: " + dt);

        //Reset the velocity timer
        vTimer.reset();

    }

    public void correctX(double newX){
        xPosPrev = xPos;
        xPos = newX;
    }

    public void correctY(double newY){
        yPosPrev = yPos;
        yPos = newY;
    }

    //Returns the raw angle provided by the imu, and flips it so negative is left
    private double getAngle(){
        return -ehr.angularOrientation;
    }

    public void stop(){
        isRunning = false;
    }

    public void setCorrectingX(boolean isCorrecting){
        correctingX = isCorrecting;
    }

    public void setCorrectingY(boolean isCorrecting){
        correctingY = isCorrecting;
    }

    public double getX(){
        return xPos;
    }

    public double getY(){
        return yPos;
    }

    public double getVelocity(){
        return velocity;
    }

    public boolean isInPositiveX(){
        return xPos > 0.0;
    }

    public double getOrientation(){
        return orientation;
    }

    @Override
    public void run() {
        elapsedTime.reset();
        while(isRunning){
            updatePosition();

            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }


}
