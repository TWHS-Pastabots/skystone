package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedStonesMovedHoriz extends PositionBasedAuton4 {

    public void setStartPos(){
        isBlue = false;
        startX = -31.0;
        startY = 8.0;
        startOrientation = -90.0;
    }

    public void drive(){
        sensing.flipAxis(); // Do because red
        if(getStoneConfig().equals("Middle") && opModeIsActive()){

            //Drive up to the middle block, turn on the intake, then drive into the middle block
            driveToPosition(-20.0, 48.0, 0.5, -90, 0.0, 0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(-25.0, 48.0, 0.50, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -25, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            //correctPosition(false, true, positioning);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 15, 10, positioning, sensing);
            //driveToPosition( 25, 21.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //driveToPosition( 25, 32.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(false, true, positioning);

            driveToPosition( -42, 32.0, 0.75, -90, 0.0, 15, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -42, 47.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( -49, 47.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -49, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 20, 10, positioning, sensing);
           // driveToPosition( 25, 28.0, 0.75, -90, 0.0, 20, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //correctPosition(false, true, positioning);
            driveToPosition( 0, 34.0, 0.3, -90, 0.0, 0, 10, positioning, sensing);

            /*
            //Drive up to the middle block, turn on the intake, then drive into the middle block
            driveToPosition(-20.0, 48.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(-23.0, 48.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -23, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            //correctPosition(false, true, positioning);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 40, 10, positioning, sensing);
            driveToPosition( 25, 25.0, 0.3, -90, 0.0, 0, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            sleep(1500);
            driveToPosition( 23, 32.0, 0.3, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(false, true, positioning);

            driveToPosition( -42, 32.0, 0.75, -90, 0.0, 40, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -42, 47.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( -47, 47.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            driveToPosition( -47, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( 23, 32.0, 0.75, -90, 0.0, 40, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //correctPosition(false, true, positioning);
            driveToPosition( 0, 34.0, 0.3, -90, 0.0, 0, 10, positioning, sensing);

             */

        }
        else if(getStoneConfig().equals("Left") && opModeIsActive()){
            //Drive up to the left block, turn on the intake, then drive into the left block
            driveToPosition(-27.0, 48.0, 0.5, -90, 0.0, 0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(-32.0, 48.0, 0.50, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -32, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            //correctPosition(false, true, positioning);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 15, 10, positioning, sensing);
            //driveToPosition( 25, 21.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //driveToPosition( 25, 32.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(false, true, positioning);

            driveToPosition( -50, 32.0, 0.75, -90, 0.0, 15, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -50, 47.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( -57, 47.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -57, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 20, 10, positioning, sensing);
            driveToPosition( 25, 28.0, 0.75, -90, 0.0, 20, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //correctPosition(false, true, positioning);
            driveToPosition( 0, 34.0, 0.3, -90, 0.0, 0, 10, positioning, sensing);

        }
        else if(getStoneConfig().equals("Right") && opModeIsActive()){
            //Drive up to the right block, turn on the intake, then drive into the right block
            driveToPosition(-13.0, 32.0, 0.5, -45, 0.0, 0, 10, positioning, sensing);
            //turn(-45.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            driveToPosition(-25.0, 44.0, 0.50, -45, 0.0, 0, 10, positioning, sensing);
            //turn(-90, TURN_SPEED, positioning);
            //correctPosition(true, true, positioning);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( -25, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            //correctPosition(false, true, positioning);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 15, 10, positioning, sensing);
           // driveToPosition( 25, 21.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Status:", "Dropping Block");
                telemetry.update();
            }
            //driveToPosition( 25, 32.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(false, true, positioning);

            driveToPosition( -34, 32.0, 0.75, -90, 0.0, 15, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -34, 47.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( -41, 47.0, 0.4, -90, 0.0, 0, 10, positioning, sensing);
            //correctPosition(true, true, positioning);
            driveToPosition( -41, 32.0, 0.5, -90, 0.0, 5, 10, positioning, sensing);
            driveToPosition( 25, 32.0, 0.75, -90, 0.0, 20, 10, positioning, sensing);
            driveToPosition( 25, 28.0, 0.75, -90, 0.0, 20, 10, positioning, sensing);
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
