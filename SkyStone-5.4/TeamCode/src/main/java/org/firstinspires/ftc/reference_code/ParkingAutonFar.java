package org.firstinspires.ftc.reference_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class ParkingAutonFar extends PositionBasedAuton {

    public void setStartPos(){
        startX = 0.0;
        startY = 0.0;
        startOrientation = 0.0;
    }

    public void drive(){
        driveToPosition(0.0, 35.0, 0.5, 0.0, 0.0, true, 4.0, positioning, sensing);
        driveToPosition(12.0, 35.0, 0.5, 0.0, 0.0, false, 4.0, positioning, sensing);
    }
}
