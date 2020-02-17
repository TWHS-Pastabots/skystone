package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueTwoStonesMovePlat extends PositionBasedAuton {

    @Override
    public void setStartPos(){
        startX = 31.0;
        startY = 8.0;
        startOrientation = 90.0;
    }

    @Override
    public void drive(){

        if(getStoneConfig().equals("Left")){
            //Drive up to the left block, turn 45 degrees, turn on the intake, then drive into the left block
            driveToPosition(16.0, 36.0, DRIVE_SPEED, 90, 0.0, 40.0, 10, positioning, sensing);
            turn(45.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            driveToPosition(20.0, 47.0, DRIVE_SPEED, 45, 0.0, 12.0, 10, positioning, sensing);

            //Turn back to 90, drive back towards the wall, drive to the platform, turn to face it, and drop the block onto it
            turn(90.0, TURN_SPEED, positioning);
            driveToPosition( 20, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -36, 32.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            turn(180, TURN_SPEED, positioning);
            driveToPosition( -36, 40.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            sensing.dropBlock();

            //Hook onto the platform, turn with it, push it into position, then unhook it
            lowerHooks();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            turn(90, TURN_SPEED_HIGH, positioning);
            driveToPosition( -44, 40.0, DRIVE_SPEED_HIGH, 90, 0.0, 30.0, 10, positioning, sensing);
            raiseHooks();

            //Drive back to the quarry, drive up to the far left block, turn on the intake, then drive into the far left block
            driveToPosition( 38, 32.0, DRIVE_SPEED, 90, 0.0, 60.0, 10, positioning, sensing);
            driveToPosition( 38, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 41, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, then drop the block onto it,
            driveToPosition( 41, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -44, 32.0, DRIVE_SPEED, 90, 0.0, 35.0, 10, positioning, sensing);
            sensing.dropBlock();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        }




        else if(getStoneConfig().equals("Middle")){
            //Drive up to the middle block, turn on the intake, then drive into the middle block
            driveToPosition(20.0, 49.0, DRIVE_SPEED, 90, 0.0, 40.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(23.0, 49.0, DRIVE_SPEED, 90, 0.0, 12.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it
            driveToPosition( 23, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -36, 32.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            turn(180, TURN_SPEED, positioning);
            driveToPosition( -36, 40.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            sensing.dropBlock();

            //Hook onto the platform, turn with it, push it into position, then unhook it
            lowerHooks();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            turn(90, TURN_SPEED_HIGH, positioning);
            driveToPosition( -44, 40.0, DRIVE_SPEED_HIGH, 90, 0.0, 30.0, 10, positioning, sensing);
            raiseHooks();

            //Drive back to the quarry, drive up to the far middle block, turn on the intake, then drive into the far middle block
            driveToPosition( 46, 32.0, DRIVE_SPEED, 90, 0.0, 60.0, 10, positioning, sensing);
            driveToPosition( 46, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 49, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, drop the block onto it,
            driveToPosition( 49, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -44, 32.0, DRIVE_SPEED, 90, 0.0, 35.0, 10, positioning, sensing);
            sensing.dropBlock();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        }




        else if(getStoneConfig().equals("Right")){
            //Drive up to the right block, turn on the intake, then drive into the right block
            driveToPosition(28.0, 49.0, DRIVE_SPEED, 90, 0.0, 40.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(31.0, 49.0, DRIVE_SPEED, 90, 0.0, 12.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 31, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -36, 32.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            turn(180, TURN_SPEED, positioning);
            driveToPosition( -36, 40.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            sensing.dropBlock();

            //Hook onto the platform, turn with it, push it into position, then unhook it
            lowerHooks();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            turn(90, TURN_SPEED_HIGH, positioning);
            driveToPosition( -44, 40.0, DRIVE_SPEED_HIGH, 90, 0.0, 30.0, 10, positioning, sensing);
            raiseHooks();

            //Drive back to the quarry, drive up to the far right block, turn on the intake, then drive into the far right block
            driveToPosition( 52, 32.0, DRIVE_SPEED, 90, 0.0, 60.0, 10, positioning, sensing);
            driveToPosition( 52, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 55, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, drop the block onto it
            driveToPosition( 55, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -44, 32.0, DRIVE_SPEED, 90, 0.0, 35.0, 10, positioning, sensing);
            sensing.dropBlock();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
        }

        telemetry.addData("Status:", "Finished Driving");
        telemetry.update();
    }

}
