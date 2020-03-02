package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueStones extends PositionBasedAuton3 {

    public void setStartPos(){
        startX = 31.0;
        startY = 8.0;
        startOrientation = 90.0;
    }

    public void drive(){
        driveToPosition(18.0, 46.0, 20, 90, 0.0, true, 10, positioning, sensing);
        sensing.activateIntake();
        driveToPosition(25.0, 46.0, 20, 90, 0.0, true, 10, positioning, sensing);
        sleep(1000);
        sensing.deActivateIntake();
        driveToPosition(25.0, 34.0, 20, 90, 0.0, true, 10, positioning, sensing);
    }
}
