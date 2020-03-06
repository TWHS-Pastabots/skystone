package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedStones extends PositionBasedAuton3 {

    public void setStartPos(){
        startX = -38.0;
        startY = 8.0;
        startOrientation = 90.0;
    }

    public void drive(){
        sensing.flipAxis(); // Do because red
        if(getStoneConfig().equals("Middle")){
            driveToPosition(-20.0, 32.0, 20, 90, 0.0, true, 10, positioning, sensing);
            turn(-90.0, TURN_SPEED, positioning);

            //Drive up to the middle block, turn on the intake, then drive into the middle block
            driveToPosition(-17.0, 48.0, 20, -90, 0.0, true, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(-25.0, 48.0, 20, -90, 0.0, true, 10, positioning, sensing);
            sleep(1000);
            sensing.deActivateIntake();

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -25, 32.0, 20, -90, 0.0, true, 10, positioning, sensing);
            driveToPosition( 24, 32.0, 20, -90, 0.0, true, 10, positioning, sensing);

        }


    }
}
