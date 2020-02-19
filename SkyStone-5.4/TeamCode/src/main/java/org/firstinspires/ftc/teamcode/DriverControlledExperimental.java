package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Driver Experimental", group="Linear OpMode")
public class DriverControlledExperimental extends OpMode{

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime= new ElapsedTime();
    double slowCon = 1.0;
    int pos = 0;

    private boolean alignRight = true;

    private DistanceSensor leftPlatformDistance;
    private DistanceSensor rightPlatformDistance;
    private DistanceSensor leftBlockDistance;
    private DistanceSensor rightBlockDistance;
    private TouchSensor touch;
    DigitalChannel magnet;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftPlatformDistance = hardwareMap.get(DistanceSensor.class, "leftPlatformDistance");
        rightPlatformDistance = hardwareMap.get(DistanceSensor.class, "rightPlatformDistance");
        leftBlockDistance = hardwareMap.get(DistanceSensor.class, "leftBlockDistance");
        rightBlockDistance = hardwareMap.get(DistanceSensor.class, "rightBlockDistance");
        touch = hardwareMap.touchSensor.get("touch");
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);

        robot.clawT.setPosition(1);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
    }

    // loop on init()
    @Override
    public void init_loop() {
    }

    // Run once on start()
    @Override
    public void start() {
        runTime.reset();
        telemetry.addData("Run Time", "reset");
        int gibbonSoundEffect = hardwareMap.appContext.getResources().getIdentifier("gibbon",
                "raw", hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, gibbonSoundEffect); //Okay now this is pure comedy

    }


    // loop on start()
    @Override
    public void loop() {

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
            //alignWithBlock();
            //correctDistanceToFoundation();
        }

        if(gamepad2.dpad_left){
            alignRight = false;
        }
        else if(gamepad2.dpad_right){
            alignRight = true;
        }


        robot.leftFront.setPower(v1*slowCon );
        robot.rightFront.setPower(v2*slowCon );
        robot.leftRear.setPower(v3*slowCon );
        robot.rightRear.setPower(v4*slowCon );

        leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
        rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
        telemetry.addData("leftD:", leftD);
        telemetry.addData("rightD:", rightD);
        telemetry.update();
        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop(){
    }

    private void alignWithFoundation(){
        double leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
        double rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
        final double TURN_POWER = 0.3;

        if(leftD > rightD){
            //Need to turn left
            while(leftD - rightD > 1.0){
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
            while(rightD - leftD > 1.0){
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

    private void correctDistanceToFoundation(){
        double dist = (leftPlatformDistance.getDistance(DistanceUnit.CM) + rightPlatformDistance.getDistance(DistanceUnit.CM)) / 2.0;
        final double DRIVE_POWER = 0.2;
        final double CORRECT_DISTANCE_CM = 3.0;

        if(dist > CORRECT_DISTANCE_CM){
            //Need to drive forward
            while(dist > CORRECT_DISTANCE_CM){
                dist = (leftPlatformDistance.getDistance(DistanceUnit.CM) + rightPlatformDistance.getDistance(DistanceUnit.CM)) / 2.0;
                robot.leftFront.setPower(DRIVE_POWER);
                robot.rightFront.setPower(DRIVE_POWER);
                robot.leftRear.setPower(DRIVE_POWER);
                robot.rightRear.setPower(DRIVE_POWER);
            }
        }
        else if(dist < CORRECT_DISTANCE_CM){
            //Need to drive back
            while(dist < CORRECT_DISTANCE_CM){
                dist = (leftPlatformDistance.getDistance(DistanceUnit.CM) + rightPlatformDistance.getDistance(DistanceUnit.CM)) / 2.0;
                robot.leftFront.setPower(-DRIVE_POWER);
                robot.rightFront.setPower(-DRIVE_POWER);
                robot.leftRear.setPower(-DRIVE_POWER);
                robot.rightRear.setPower(-DRIVE_POWER);
            }
        }

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
    }

    private void alignWithBlock(){
        double leftD = leftBlockDistance.getDistance(DistanceUnit.CM);
        double rightD = rightBlockDistance.getDistance(DistanceUnit.CM);
        final double DRIVE_POWER = 0.25;

        while(Math.abs(leftD - rightD) < 5.0){
            leftD = leftPlatformDistance.getDistance(DistanceUnit.CM);
            rightD = rightPlatformDistance.getDistance(DistanceUnit.CM);
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

}

