package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkingAuton extends PositionBasedAuton3 {

    public void setStartPos(){
        startX = 31.0;
        startY = 8.0;
        startOrientation = 90.0;
    }

    public void drive(){
        driveToPosition(0.0, 48.0, 20, 0.0, 0, false, 3.5, positioning, sensing);
        turn(90, 0.75, positioning);
    }
}
