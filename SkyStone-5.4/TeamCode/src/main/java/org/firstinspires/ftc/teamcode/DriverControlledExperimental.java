package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name="Driver Experimental", group="Linear OpMode")
public class DriverControlledExperimental extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime= new ElapsedTime();
    double slowCon = 1.0;
    int pos = 0;

    public boolean alignRight = true;

    private DistanceSensor leftPlatformDistance;
    private DistanceSensor rightPlatformDistance;
    private DistanceSensor armHeightDistance;
    private TouchSensor touch;
    DigitalChannel magnet;

    public ServoControl servoControl;

    private ArrayList<Integer> liftPositions;
    private int selectedLiftPosition = 0;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftPlatformDistance = hardwareMap.get(DistanceSensor.class, "leftPlatformDistance");
        rightPlatformDistance = hardwareMap.get(DistanceSensor.class, "rightPlatformDistance");
        armHeightDistance = hardwareMap.get(DistanceSensor.class, "armHeightDistance");
        touch = hardwareMap.touchSensor.get("touch");
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.clawT.setPosition(1);

        servoControl = new ServoControl();
        Thread servoControlThread = new Thread(servoControl);
        servoControlThread.start();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runTime.reset();

        while(opModeIsActive()){
            double G1leftStickX = -gamepad1.right_stick_x;
            double G1leftStickY = -gamepad1.right_stick_y;
            double turnCon = gamepad1.left_stick_x;
            boolean G1a = gamepad1.a;
            boolean G1b = gamepad1.b;
            boolean G2a = gamepad2.a;
            boolean G2b = gamepad2.b;
            boolean G2x = gamepad2.x;
            boolean G2y = gamepad2.y;
            boolean G1y = gamepad1.y;
            double G2leftStickY = -gamepad2.left_stick_y;
            boolean G2rb = gamepad2.right_bumper;
            boolean G2lb = gamepad2.left_bumper;
            boolean G1down = gamepad1.dpad_down;
            boolean G1up = gamepad1.dpad_up;
            boolean G2up = gamepad2.dpad_up;
            boolean G2down = gamepad2.dpad_down;
            boolean G2left = gamepad2.dpad_left;
            boolean G2right = gamepad2.dpad_right;
            // example how to use buttons
            double radius = Math.hypot( G1leftStickX, G1leftStickY);
            double ang = Math.atan2( G1leftStickY, G1leftStickX) - Math.PI/4;
            double v1 = radius * Math.cos(ang) + turnCon;
            double v2 = radius * Math.sin(ang) - turnCon;
            double v3 = radius * Math.sin(ang) + turnCon;
            double v4 = radius * Math.cos(ang) - turnCon;
            double leftD;
            double rightD;

            if (G1b){
                slowCon = .3;

            }
            if (G1a){
                slowCon = 1.0;
            }

            // intake code
            if(gamepad1.right_bumper){
                robot.leftIn.setPower(-.7);
                robot.rightIn.setPower(-.7);
            }
            else if(gamepad1.left_bumper){
                robot.leftIn.setPower(.7);
                robot.rightIn.setPower(.7);
            }
            else {
                robot.leftIn.setPower(0);
                robot.rightIn.setPower(0);
            }

            if(gamepad1.x){
                alignWithFoundation();
                correctDistanceToFoundation(2.0);
                alignWithFoundation();
            }

            if(gamepad2.dpad_left){
                alignRight = false;
            }
            else if(gamepad2.dpad_right){
                alignRight = true;
            }

            if(gamepad2.a){
                servoControl.raiseLift();
            }

            if(gamepad2.dpad_up && selectedLiftPosition < 10)
                selectedLiftPosition++;
            else if(gamepad2.dpad_down && selectedLiftPosition > 0)
                selectedLiftPosition--;


            robot.leftFront.setPower(v1*slowCon );
            robot.rightFront.setPower(v2*slowCon );
            robot.leftRear.setPower(v3*slowCon );
            robot.rightRear.setPower(v4*slowCon );

            leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
            rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
            telemetry.addData("Lift Motor Encoder: " , robot.liftMotor.getCurrentPosition());
            telemetry.addData("leftD:", leftD);
            telemetry.addData("rightD:", rightD);

            telemetry.addData("Height:", armHeightDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
            telemetry.update();
        }

        servoControl.stop();
    }

    private void alignWithFoundation(){
        double leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
        double rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
        final double TURN_POWER = 0.2;

        if(leftD > rightD){
            //Need to turn left
            while(leftD - rightD > 0.25){
                leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
                rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
                robot.leftFront.setPower(-TURN_POWER);
                robot.rightFront.setPower(TURN_POWER);
                robot.leftRear.setPower(-TURN_POWER);
                robot.rightRear.setPower(TURN_POWER);
                telemetry.addData("leftD:", leftD);
                telemetry.addData("rightD:", rightD);
                telemetry.update();
            }
        }
        else if(rightD > leftD){
            //Need to turn right
            while(rightD - leftD > 0.25){
                leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
                rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
                robot.leftFront.setPower(TURN_POWER);
                robot.rightFront.setPower(-TURN_POWER);
                robot.leftRear.setPower(TURN_POWER);
                robot.rightRear.setPower(-TURN_POWER);
                telemetry.addData("leftD:", leftD);
                telemetry.addData("rightD:", rightD);
                telemetry.update();
            }
        }

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
    }

    private void correctDistanceToFoundation(double correctDistance){
        double dist = (leftPlatformDistance.getDistance(DistanceUnit.CM) + rightPlatformDistance.getDistance(DistanceUnit.CM)) / 2.0;
        final double DRIVE_POWER = 0.15;

        if(dist > correctDistance){
            //Need to drive backward
            while(dist > correctDistance){
                dist = (leftPlatformDistance.getDistance(DistanceUnit.CM) + rightPlatformDistance.getDistance(DistanceUnit.CM)) / 2.0;
                robot.leftFront.setPower(-DRIVE_POWER);
                robot.rightFront.setPower(-DRIVE_POWER);
                robot.leftRear.setPower(-DRIVE_POWER);
                robot.rightRear.setPower(-DRIVE_POWER);
            }
        }
        else if(dist < correctDistance){
            //Need to drive forward
            while(dist < correctDistance){
                dist = (leftPlatformDistance.getDistance(DistanceUnit.CM) + rightPlatformDistance.getDistance(DistanceUnit.CM)) / 2.0;
                robot.leftFront.setPower(DRIVE_POWER);
                robot.rightFront.setPower(DRIVE_POWER);
                robot.leftRear.setPower(DRIVE_POWER);
                robot.rightRear.setPower(DRIVE_POWER);
            }
        }

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
    }

    /*
    private void alignWithBlock(){
        double leftD = leftBlockDistance.getDistance(DistanceUnit.CM) - 3; //Subtract due to the way the sensors are mounted on the physical robot
        double rightD = rightBlockDistance.getDistance(DistanceUnit.CM);
        final double DRIVE_POWER = 0.25;

        while(Math.abs(leftD - rightD) > 3.0 || leftD > 20.0 || rightD > 20.0 ){
            leftD = leftBlockDistance.getDistance(DistanceUnit.CM);
            rightD = rightBlockDistance.getDistance(DistanceUnit.CM);
            if(alignRight){
                robot.leftFront.setPower(DRIVE_POWER);
                robot.rightFront.setPower(-DRIVE_POWER);
                robot.leftRear.setPower(-DRIVE_POWER);
                robot.rightRear.setPower(DRIVE_POWER);
            }
            else if(!alignRight){
                robot.leftFront.setPower(-DRIVE_POWER);
                robot.rightFront.setPower(DRIVE_POWER);
                robot.leftRear.setPower(DRIVE_POWER);
                robot.rightRear.setPower(-DRIVE_POWER);
            }
        }

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

    }

     */

    public class ServoControl implements Runnable{

        private boolean isRunning = true;
        private boolean dropBlockRequested = false;
        private boolean raiseLiftActivated = false;
        private boolean lowerLiftActivated = false;
        private boolean blockDropped = false;
        double pulleyCircumference = 2 * Math.PI * 1.0;
        double liftMotorGearRatio = 70.0 / 56.0;
        double liftEncoderCountsPerInch = (2240 * pulleyCircumference * liftMotorGearRatio) / 3.0;
        double targetLiftPosition = 1500; //liftPositions.get(selectedLiftPosition) //liftEncoderCountsPerInch * 4.0


        public boolean isBlockDropped(){
            return blockDropped;
        }

        public void stop(){
            isRunning = false;
        }

        public void dropBlock(){
            dropBlockRequested = true;
        }

        public void raiseLift(){
            raiseLiftActivated = true;
        }

        public void lowerLift(){
            lowerLiftActivated = true;
        }


        private void closeClaw(){
            robot.claw.setPosition(1.0);
        }

        private void openClaw(){
            robot.claw.setPosition(0.0);
        }

        private void turnClawIn(){
            robot.clawT.setPosition(1.0);
        }

        private void turnClawOut(){
            robot.clawT.setPosition(0.0);
        }

        private void activateSpanker(){
            robot.servo_blockPush.setPosition(0.0);
        }

        private void retractSpanker(){
            robot.servo_blockPush.setPosition(1.0);
        }

        private void sleep(int timeMs){
            try {
                Thread.sleep(timeMs);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void run(){
            while(isRunning){

                if(robot.blockInSensor.getDistance(DistanceUnit.INCH) > 0.5){
                    activateSpanker();
                    sleep(1000);
                    closeClaw();
                }
                else
                    retractSpanker();

                if(raiseLiftActivated){
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //Grab the block with the claw and raise the lift into position then turn the claw out
                    while(Math.abs(robot.liftMotor.getCurrentPosition()) < targetLiftPosition) {
                        robot.liftMotor.setPower(-0.75);
                    }
                    robot.liftMotor.setPower(0);
                    //retractSpanker();

                    //Wait until dropping the block is requested, then let go of the block and turn the claw back in
                    while(!blockDropped){
                        if(dropBlockRequested){
                            turnClawOut();
                            sleep(2000);
                            openClaw();
                            sleep(500);
                            blockDropped = true;
                        }
                    }

                    dropBlockRequested = false;
                    blockDropped = false;
                    raiseLiftActivated = false;
                }

                if(lowerLiftActivated){
                    closeClaw();
                    turnClawIn();
                    sleep(750);
                    //Lower the lift back down and open the claw
                    while(!touch.isPressed()){
                        robot.liftMotor.setPower(0.15);
                    }
                    robot.liftMotor.setPower(0.0);
                    openClaw();

                    robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lowerLiftActivated = false;

                }

            }
        }
    }

}

