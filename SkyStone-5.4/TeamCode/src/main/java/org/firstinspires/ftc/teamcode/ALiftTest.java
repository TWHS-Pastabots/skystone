package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class ALiftTest extends PositionBasedAuton3 {

    public void setStartPos(){
        startX = -24.0;
        startY = 8.0;
        startOrientation = 90.0;
    }

    public void drive(){
        sensing.activateIntake();
        sleep(1000);
        sensing.deActivateIntake();
        sleep(10000);
        sensing.dropBlock();
        sleep(10000);

    }
}
