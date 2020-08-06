package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedFoundationVertParkLow extends PositionBasedAuton4 {

    public void setStartPos(){
        isBlue = false;
        startX = 39.0;
        startY = 10.0;
        startOrientation = -179.0;
    }

    public void drive(){
        driveToPosition(39.0, 38.0, 0.4, -179, 0.0, 0.0, 5.0, positioning, sensing);
        lowerHooks();
        sleep(1500);
        driveToPosition(39.0, 15.0, 1.0, -179, 0.0, 0.0, 5.0, positioning, sensing);
        turnNoRampDown(-90, 0.75, positioning);
        raiseHooks();
        sleep(500);
        driveToPosition(positioning.getX(), 15.0, 0.8, -90, 0.0, 0.0, 5.0, positioning, sensing);
        driveToPosition(45.0, positioning.getY(), 0.8, -90, 0.0, 0.0, 5.0, positioning, sensing);
        driveToPosition(0.0, 10.0, 0.4, -90, 0.0, 0.0, 5.0, positioning, sensing);

    }
}
