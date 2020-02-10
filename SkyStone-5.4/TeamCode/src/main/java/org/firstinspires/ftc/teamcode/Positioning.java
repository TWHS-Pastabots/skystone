package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Positioning implements Runnable {

    //The amount of encoder ticks for each inch the robot moves.
    final double COUNTS_PER_INCH = 1141.9488791276;

    private BNO055IMU imu;

    private DcMotor leftEnc;
    private DcMotor rightEnc;
    private DcMotor horizEnc;

    private double previousVerticalLeftEncoderTickOffsetPerDegree;
    private double reviousVerticalRightEncoderTickOffsetPerDegree;
    private double previousHorizontalEncoderTickOffsetPerDegree;

    private double verticalLeftEncoderTickOffsetPerDegree;
    private double verticalRightEncoderTickOffsetPerDegree;
    private double horizontalEncoderTickOffsetPerDegree;

    private double previousRightEncoderPosition = 0, previousLeftEncoderPosition = 0, previousHorizEncoderPosition = 0;
    private double verticalRightEncoderPosition = 0, verticalLeftEncoderPosition = 0, horizontalEncoderPosition = 0;


    private double orientation;
    private double previousOrientation;
    private double rawOrientationPrevious = 0.0;
    private double rawOrientation;

    private double xPos;
    private double yPos;



    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private File verticalLeftTickOffestFile = AppUtil.getInstance().getSettingsFile("verticalLeftTickOffset.txt");
    private File verticalRightTickOffestFile = AppUtil.getInstance().getSettingsFile("verticalRightTickOffset.txt");

    //Thead run condition
    private boolean isRunning = true;




    private int sleepTime;

    RobotHardware robot = new RobotHardware();

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param leftEnc left odometry encoder, facing the vertical direction
     * @param rightEnc right odometry encoder, facing the vertical direction
     * @param horizEnc horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
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


        this.verticalLeftEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(verticalLeftTickOffestFile).trim());
        this.verticalRightEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(verticalRightTickOffestFile).trim());
        this.horizontalEncoderTickOffsetPerDegree = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    private void updatePosition () {
        updateOrientation();
        verticalLeftEncoderPosition = leftEnc.getCurrentPosition();
        verticalRightEncoderPosition = rightEnc.getCurrentPosition();
        horizontalEncoderPosition = horizEnc.getCurrentPosition();

        double orientationChange = orientation - previousOrientation;
        double leftChange = (verticalLeftEncoderPosition - previousLeftEncoderPosition) - (orientationChange * verticalLeftEncoderTickOffsetPerDegree);
        double rightChange = (verticalRightEncoderPosition - previousRightEncoderPosition) - (orientationChange * verticalRightEncoderTickOffsetPerDegree);
        double horizontalChange = (horizontalEncoderPosition - previousHorizEncoderPosition) - (orientationChange * horizontalEncoderTickOffsetPerDegree);
        double verticalChange = (leftChange + rightChange) / 2.0;



        previousOrientation = orientation;
    }

    private void updateOrientation(){
        rawOrientation = getAngle();
        double rawOrientationChange = rawOrientation - rawOrientationPrevious;

        if(rawOrientationChange < -180)
            rawOrientationChange += 360;
        else if(rawOrientationChange > 180)
            rawOrientationChange -=360;

        rawOrientationPrevious = rawOrientation;
        orientation += rawOrientationChange;
    }

    private double getAngle(){
       return -imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void run() {

    }


}
