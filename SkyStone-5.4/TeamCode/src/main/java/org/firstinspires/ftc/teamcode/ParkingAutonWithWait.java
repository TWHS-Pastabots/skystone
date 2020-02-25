package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkingAutonWithWait extends PositionBasedAuton {

    public void setStartPos(){
        startX = 0.0;
        startY = 0.0;
        startOrientation = 0.0;
    }

    public void drive(){
        sleep(23000);
        driveToPosition(0.0, 6.0, 0.5, 0.0, 0.0, false, 4.0, positioning, sensing);
    }
}
