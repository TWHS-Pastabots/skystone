package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkingAuton extends PositionBasedAuton3 {

    public void setStartPos(){
        startX = 0.0;
        startY = 0.0;
        startOrientation = 0.0;
    }

    public void drive(){
        driveToPosition(0.0, 96.0, 40, 0.0, 0, true, 3.5, 0.2,10.0, positioning, sensing);
        sleep(2000);
        turn(90, 0.75, positioning);
    }
}
