package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedStonesFast extends PositionBasedAuton4 {

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
            driveToPosition(-20.0, 48.0, 0.75, -90, 0.0, 0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(-23.0, 48.0, 0.60, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -23, 32.0, 0.75, -90, 0.0, 10, 10, positioning, sensing);
            //correctPosition(false, true, positioning);
            driveToPosition( 25, 32.0, 1.0, -90, 0.0, 20, 10, positioning, sensing);
            driveToPosition( 25, 25.0, 0.5, -90, 0.0, 0, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            driveToPosition( 23, 32.0, 0.5, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(false, true, positioning);

            driveToPosition( -42, 32.0, 1.0, -90, 0.0, 20, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -42, 47.0, 0.75, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( -47, 47.0, 0.6, -90, 0.0, 0, 10, positioning, sensing);
            driveToPosition( -47, 32.0, 0.75, -90, 0.0, 5, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( 23, 32.0, 1.0, -90, 0.0, 20, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //correctPosition(false, true, positioning);
            driveToPosition( 0, 34.0, 0.3, -90, 0.0, 0, 10, positioning, sensing);


        }


    }
}
