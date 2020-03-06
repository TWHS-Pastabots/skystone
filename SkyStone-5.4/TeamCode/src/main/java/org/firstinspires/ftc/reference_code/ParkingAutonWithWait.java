package org.firstinspires.ftc.reference_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
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
