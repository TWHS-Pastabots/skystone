package org.firstinspires.ftc.reference_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueTwoStones2 extends PositionBasedAuton2 {

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
            driveToPosition(13.0, 36.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(45.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            driveToPosition(20.0, 47.0, DRIVE_SPEED, 45, 0.0, true, 10, positioning, sensing);

            //Turn back to 90, drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            turn(90.0, TURN_SPEED, positioning);
            driveToPosition( 20, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( -24, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(0.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            sleep(2000);
            turn(90, TURN_SPEED, positioning);

            //Drive back to the quarry, drive up to the far left block, turn on the intake, then drive into the far left block
            driveToPosition( 34, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( 34, 48.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 41, 48.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 41, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( -24, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(0.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            sleep(2000);
            turn(90, TURN_SPEED, positioning);

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);

        }




        else if(getStoneConfig().equals("Middle")){
            //Drive up to the middle block, turn on the intake, then drive into the middle block
            driveToPosition(17.0, 48.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(25.0, 48.0, 0.5, 90, 0.0, true, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 25, 32.0, 0.5, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( -24, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(0.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            sleep(2000);
            turn(90, TURN_SPEED, positioning);

            //Drive back to the quarry, drive up to the far middle block, turn on the intake, then drive into the far middle block
            driveToPosition( 42, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( 42, 48.0, 0.5, 90, 0.0, true, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 49, 48.0, 0.5, 90, 0.0, true, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 49, 3420, 0.5, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( -24, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(0.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            sleep(2000);
            turn(90, TURN_SPEED, positioning);

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
        }




        else if(getStoneConfig().equals("Right")){
            //Drive up to the right block, turn on the intake, then drive into the right block
            driveToPosition(25.0, 48.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(30.0, 48.0, 0.5, 90, 0.0, true, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 30, 32.0, 0.5, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( -24, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(0.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            sleep(2000);
            turn(90, TURN_SPEED, positioning);

            //Drive back to the quarry, drive up to the far middle block, turn on the intake, then drive into the far middle block
            driveToPosition( 50, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( 50, 48.0, 0.5, 90, 0.0, true, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 57, 48.0, 0.5, 90, 0.0, true, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 57, 32.0, 0.5, 90, 0.0, true, 10, positioning, sensing);
            driveToPosition( -24, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);
            turn(0.0, TURN_SPEED, positioning);
            sensing.activateIntake();
            sleep(2000);
            turn(90, TURN_SPEED, positioning);

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, true, 10, positioning, sensing);

            /*
            //Drive up to the right block, turn on the intake, then drive into the right block
            driveToPosition(28.0, 49.0, DRIVE_SPEED, 90, 0.0, 40.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition(31.0, 49.0, DRIVE_SPEED, 90, 0.0, 12.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 31, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -36, 32.0, DRIVE_SPEED, 90, 0.0, 30.0, 10, positioning, sensing);
            turn(180, TURN_SPEED, positioning);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Motor Enc Pos:", sensing.getEncPos());
                telemetry.update();
            }
            turn(90, TURN_SPEED, positioning);

            //Drive back to the quarry, drive up to the far right block, turn on the intake, then drive into the far right block
            driveToPosition( 52, 32.0, DRIVE_SPEED, 90, 0.0, 60.0, 10, positioning, sensing);
            driveToPosition( 52, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            sensing.activateIntake();
            driveToPosition( 55, 49.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);

            //Drive back towards the wall, drive to the platform, turn to face it, drop the block onto it, then turn back to face the quarry
            driveToPosition( 55, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);
            driveToPosition( -36, 32.0, DRIVE_SPEED, 90, 0.0, 35.0, 10, positioning, sensing);
            turn(180, TURN_SPEED, positioning);
            sensing.dropBlock();
            while(!sensing.isBlockDropped()){
                telemetry.addData("Motor Enc Pos:", sensing.getEncPos());
                telemetry.update();
            }
            turn(90, TURN_SPEED, positioning);

            //Drive to park under the skybridge, far from the wall
            driveToPosition( 0, 32.0, DRIVE_SPEED, 90, 0.0, 20.0, 10, positioning, sensing);

             */
        }

        telemetry.addData("Status:", "Finished Driving");
        telemetry.update();


    }

}
