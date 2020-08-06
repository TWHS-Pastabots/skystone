package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.concurrent.CountDownLatch;

public class ExpansionHubReading implements Runnable {
    private List<LynxModule> allHubs;
    private boolean isRunning = true;
    private RobotHardware robot;

    private boolean readHeight = false;
    private boolean readBlock = false;
    private boolean readCorrections = false;

    private boolean updateFlag = false;

    private int readingID = 0;

    public volatile double leftEnc, rightEnc, horizEnc; //Encoder positions
    public volatile double angularOrientation; // IMU Angular Orientation
    public volatile double leftFrontPow, leftRearPow, rightFrontPow, rightRearPow, leftInPow, rightInPow; //Drive and intake motor powers
    public volatile double armHeightD, blockD, leftD, frontD; //Distance Sensor distances, in inches


    public ExpansionHubReading(RobotHardware robot, List<LynxModule> allHubs){
        this.allHubs = allHubs;
        this.robot = robot;
    }

    public void stop(){
        isRunning = false;
    }

    private void updateReadings(){
        readingID++;
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        //Update encoder positions
        leftEnc = robot.leftEnc.getCurrentPosition();
        rightEnc = robot.rightEnc.getCurrentPosition();
        horizEnc = robot.horizEnc.getCurrentPosition();

        //Update angle
        angularOrientation = robot.imu.getAngularOrientation().firstAngle;

        //Update motor powers
        leftFrontPow = robot.leftFront.getPower();
        leftRearPow = robot.leftRear.getPower();
        rightFrontPow = robot.rightFront.getPower();
        rightRearPow = robot.rightRear.getPower();
        leftInPow = robot.leftIn.getPower();
        rightInPow = robot.rightIn.getPower();

        //Update sensor distances
        if(readHeight)
            armHeightD = robot.armHeightDistance.getDistance(DistanceUnit.INCH);
        if(readBlock)
            blockD = robot.blockDistance.getDistance(DistanceUnit.INCH);
        if(readCorrections){
            leftD = robot.leftDistance.getDistance(DistanceUnit.INCH);
            frontD = robot.frontDistance.getDistance(DistanceUnit.INCH);
        }
    }

    public void setReadHeight(boolean setBool){
        readHeight = setBool;
    }
    public void setReadBlock(boolean setBool){
        readBlock = setBool;
    }
    public void setReadCorrections(boolean setBool){
        readCorrections = setBool;
    }

    public boolean getUpdateFlag(){
        return updateFlag;
    }

    public int getReadingID(){
        return readingID;
    }

    public void waitForNextReading(){
        boolean startCon = getUpdateFlag();
        int count = 0;
        while(count < 1){
            if(getUpdateFlag() != startCon){
                count++;
            }
        }
    }

    @Override
    public void run(){
        while(isRunning){
            updateReadings();
            updateFlag = !updateFlag;
        }
    }
}
