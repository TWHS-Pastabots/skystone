package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.awt.font.NumericShaper;

@TeleOp(name="Driver", group="Linear OpMode")
public class DriverControlled extends OpMode{

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime= new ElapsedTime();
    double slowCon = 1.0;
    int pos = 0;

    private DistanceSensor distance;
    private TouchSensor touch;
    DigitalChannel magnet;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distance = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distance;
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
    }                                                                                      //Sound effect will play ONCE upon pressing play

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
        // Sets power of motor, spins wheels

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

        // lift code
        if (G2up && magnet.getState())
        {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setPower(-0.5);
            pos = robot.liftMotor.getCurrentPosition();
        }
        else if (G2down && !touch.isPressed())
        {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setPower(0.2);
            pos = robot.liftMotor.getCurrentPosition();
        }
        else
        {
            robot.liftMotor.setTargetPosition(pos);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor.setPower(1);
            telemetry.addData("Staying at:", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }

        //servos
        if (G2rb)
        {
            robot.leftH.setPosition(0);
            robot.rightH.setPosition(1);
        }
        else if (G2lb)
        {
            robot.leftH.setPosition(1);
            robot.rightH.setPosition(0);
        }
        // claw
        if (G2a)
        {
            robot.claw.setPosition(1);
        }
        else if (G2b)
        {
            robot.claw.setPosition(0);
        }

        if(G2right)
        {
            robot.clawT.setPosition(0);
        }
        else if (G2left)
        {
            robot.clawT.setPosition(1);
        }

        /*if (G1leftStickY>.9){
            v1=1;
            v2=1;
            v3=1;
            v4=1;
        }
        if (G1leftStickY<-.9){
            v1=-1;
            v2=-1;
            v3=-1;
            v4=-1;
        }

        if (G1leftStickY<-.9){
            v1=-1;
            v2=-1;
            v3=-1;
            v4=-1;
        }
        if (G1leftStickY>.9){
            v1=1;
            v2=1;
            v3=1;
            v4=1;
        }
        if (G1leftStickX>.9){
            v1=1;
            v2=-1;
            v3=-1;
            v4=1;
        }
        if (G1leftStickX<-.9){
            v1=-1;
            v2=1;
            v3=1;
            v4=-1;
        }

         */

        robot.leftFront.setPower(v1*slowCon );
        robot.rightFront.setPower(v2*slowCon );
        robot.leftRear.setPower(v3*slowCon );
        robot.rightRear.setPower(v4*slowCon );

        telemetry.addData("Powers:", v1);
        telemetry.addData("", v2);
        telemetry.addData("", v3);
        telemetry.addData("", v4);
        telemetry.addData("Magnet:", magnet.getState());
        //telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop(){
    }

    //test 15
    //Another one
}

