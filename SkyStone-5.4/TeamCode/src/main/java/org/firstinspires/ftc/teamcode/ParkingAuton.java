package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkingAuton extends PositionBasedAuton4 {

    public void setStartPos(){
        startX = 0;
        startY = 0;
        startOrientation = 0;
    }

    public void drive(){
        driveToPosition(0.0, 48.0, 0.4, 0.0, 0.0, 0, 10, positioning, sensing);
    }
}
