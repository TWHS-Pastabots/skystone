package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedStones extends PositionBasedAuton3 {

    public void setStartPos(){
        isBlue = false;
        startX = -31.0;
        startY = 8.0;
        startOrientation = -90.0;
    }

    public void drive(){
        sensing.flipAxis(); // Do because red
        if(getStoneConfig().equals("Middle")){
            //driveToPosition(-20.0, 28.0, 20, 90, 0.0, true, 10, positioning, sensing);
            //turn(-90.0, TURN_SPEED, positioning);

            //Drive up to the middle block, turn on the intake, then drive into the middle block
            driveToPosition(-20.0, 45.0, 25, -90, 0.0, 0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(-24.0, 45.0, 15, -90, 0.0, 0, 10, positioning, sensing);
            correctPosition(true, true, positioning);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -30, 32.0, 25, -90, 0.0, 5, 10, positioning, sensing);
            correctPosition(false, true, positioning);
            driveToPosition( 20, 32.0, 30, -90, 0.0, 20, 10, positioning, sensing);
            driveToPosition( 20, 25.0, 15, -90, 0.0, 0, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            sleep(1500);
            driveToPosition( 20, 32.0, 20, -90, 0.0, 0, 10, positioning, sensing);
            correctPosition(false, true, positioning);

            driveToPosition( -42, 32.0, 30, -90, 0.0, 15, 10, positioning, sensing);
            correctPosition(true, true, positioning);
            driveToPosition( -42, 47.0, 25, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( -48, 47.0, 20, -90, 0.0, 0, 10, positioning, sensing);
            driveToPosition( -48, 32.0, 20, -90, 0.0, 0, 10, positioning, sensing);
            correctPosition(true, true, positioning);
            driveToPosition( 23, 32.0, 25, -90, 0.0, 15, 4, positioning, sensing);
            correctPosition(false, true, positioning);
            //driveToPosition( 24, 28.0, 15, -90, 0.0, false, 3, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            driveToPosition( 0, 32.0, 10, -90, 0.0, 0, 10, positioning, sensing);

        }


    }
}
