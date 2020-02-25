package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkingAuton extends PositionBasedAuton {

    public void setStartPos(){
        startX = 0.0;
        startY = 0.0;
        startOrientation = 0.0;
    }

    public void drive(){
        driveToPosition(0.0, 24.0, 0.5, 0.0, 0.0, false, 4.0, positioning, sensing);
        sleep(2000);
        driveToPosition(12.0, 24.0, 0.5, 0.0, 0.0, false, 4.0, positioning, sensing);
    }
}
