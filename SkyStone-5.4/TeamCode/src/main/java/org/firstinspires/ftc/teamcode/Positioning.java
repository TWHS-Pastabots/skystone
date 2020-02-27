package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Cam and Matthew on 02102020.
 * Example OpMode that runs the Positioning thread and accesses the (x, y, theta) coordinate values
 */

public class Positioning implements Runnable {

    //The amount of encoder ticks for each inch the robot moves.
    private final double COUNTS_PER_INCH = 1084.85144; //1141.9488791276

    //Motors and inertial measurement unit
    private BNO055IMU imu;
    private DcMotor leftEnc;
    private DcMotor rightEnc;
    private DcMotor horizEnc;

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

    //The current velocity of the robot, in inches per second
    private double velocity;
    private ElapsedTime vTimer = new ElapsedTime();

    //Files used for retrieving the tick offsets
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private File verticalLeftTickOffestFile = AppUtil.getInstance().getSettingsFile("verticalLeftTickOffset.txt");
    private File verticalRightTickOffestFile = AppUtil.getInstance().getSettingsFile("verticalRightTickOffset.txt");

    //Thread run condition
    private boolean isRunning = true;

    //Interval between each run of the thread
    private int sleepTime;

    /**
     * Constructor for Positioning Thread
     * @param leftEnc left odometry encoder, facing the vertical direction
     * @param rightEnc right odometry encoder, facing the vertical direction
     * @param horizEnc horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the Positioning thread (50-75 milliseconds is suggested)
     */
    public Positioning(DcMotor leftEnc, DcMotor rightEnc, DcMotor horizEnc, int threadSleepDelay, BNO055IMU imu, double startOrientation, double startX, double startY){
        this.leftEnc    = leftEnc;
        this.rightEnc   = rightEnc;
        this.horizEnc   = horizEnc;
        this.imu        = imu;
        sleepTime = threadSleepDelay;
        this.orientation = startOrientation;
        this.previousOrientation = startOrientation;
        this.xPos = startX;
        this.yPos = startY;
        this.xPosPrev = startX;
        this.yPosPrev = startY;


        this.verticalLeftEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(verticalLeftTickOffestFile).trim());
        this.verticalRightEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(verticalRightTickOffestFile).trim());
        this.horizontalEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    //Updates the x and y positions of the robot
    private void updatePosition () {
        //Find the current values for the encoders and orientation
        updateOrientation();
        updateVelocity();
        verticalLeftEncoderPosition = leftEnc.getCurrentPosition();
        verticalRightEncoderPosition = rightEnc.getCurrentPosition();
        horizontalEncoderPosition = horizEnc.getCurrentPosition();

        //Calculate the changes in each encoder as well as the orientation
        double orientationChange = orientation - previousOrientation;
        double leftChange = (verticalLeftEncoderPosition - previousLeftEncoderPosition) - (orientationChange * verticalLeftEncoderTickOffsetPerDegree);
        double rightChange = (verticalRightEncoderPosition - previousRightEncoderPosition) - (orientationChange * verticalRightEncoderTickOffsetPerDegree);
        double horizontalChange = (horizontalEncoderPosition - previousHorizEncoderPosition) - (orientationChange * horizontalEncoderTickOffsetPerDegree);
        double h = horizontalChange;
        double v = (leftChange + rightChange) / 2.0;
        double orientationRadians = Math.toRadians(orientation);

        //Calculate and update the position
        xPosPrev = xPos;
        yPosPrev = yPos;
        xPos += (v*Math.sin(orientationRadians) + h*Math.cos(orientationRadians)) / COUNTS_PER_INCH;
        yPos += (v*Math.cos(orientationRadians) - h*Math.sin(orientationRadians)) / COUNTS_PER_INCH;

        //Set up each of the previous values to be ready for the next iteration
        previousLeftEncoderPosition = verticalLeftEncoderPosition;
        previousRightEncoderPosition = verticalRightEncoderPosition;
        previousHorizEncoderPosition = horizontalEncoderPosition;
        previousOrientation = orientation;
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
        double dt = vTimer.seconds();

        //Calculate the velocity as the overall change in distance over the time the move took
        velocity = Math.hypot(deltaX, deltaY) / dt;

        //Reset the velocity timer
        vTimer.reset();

    }

    //Returns the raw angle provided by the imu, and flips it so negative is left
    private double getAngle(){
        return -imu.getAngularOrientation().firstAngle;
    }

    public void stop(){
        isRunning = false;
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
