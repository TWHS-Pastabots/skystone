package DeprecatedCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp
@Disabled
public class StoneDetectOp extends LinearOpMode {
    private DcMotor motor_center;
    private DcMotor motor_left;
    private DcMotor motor_right;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean fineControlMode;
    private ElapsedTime aTimer = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static final String VUFORIA_KEY = "AT27Zpn/////AAABma7sU1s4XEtftm65yZXkRCWM9i6m/6wUIGvUsFBSzT7L9tlAhl6/ckO7gKvgbCNwRbANbj561XOWx1QuHiRnbSz3JeukZwsEkdkEANU7lroQ30T/QadU3YNQlSL09JazvYxX2UkXiVBNnadlECH+4/mBqUNGCHsKnyH5hVArtcjRsSSc3B5GuFqqO8zD35HW3ot0pPMsoFuJA7XyXhx9N2DcVxGdAZWhtqLQw91ZymFpOAT6YIxQcQfqu2BOuvU0y7Z2wjlE9BvVD/BXFNQreloZlan8LZT+NTqinwlSEwjvL6Y4v5VDcoAR9xRHBmzKlin75QH/WoRR+S1Z3pF6lUahubM9/ZnGFb/WSMpOjiEi";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        motor_center = hardwareMap.get(DcMotor.class, "motor_center");
        motor_left = hardwareMap.get(DcMotor.class, "motor_left");
        motor_right = hardwareMap.get(DcMotor.class, "motor_right");

        fineControlMode = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_center.setDirection(DcMotor.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                    }
                    //Checks to see which pattern is present in the initial quarry
                    if(updatedRecognitions != null && updatedRecognitions.size() == 3){
                        ArrayList<Integer> stones = new ArrayList<>();
                        int skyStoneX = -1;
                        int stone1X = -1;
                        int stone2X = -1;
                        for(Recognition recognition : updatedRecognitions){
                            if(recognition.getLabel().equals(LABEL_STONE) && stone1X == -1)
                                stone1X = (int)recognition.getLeft();
                            else if(recognition.getLabel().equals(LABEL_STONE) && stone2X == -1)
                                stone2X = (int)recognition.getLeft();
                            else if(recognition.getLabel().equals(LABEL_SKYSTONE) && skyStoneX == -1)
                                skyStoneX = (int)recognition.getLeft();
                        }
                        if(skyStoneX != -1 && stone1X != -1 && stone2X != -1){
                            if(skyStoneX > stone1X && skyStoneX > stone2X)
                                telemetry.addData("Pattern", "C");
                            else if(skyStoneX < stone1X && skyStoneX < stone2X)
                                telemetry.addData("Pattern", "A");
                            else
                                telemetry.addData("Pattern", "B");
                        }
                    }
                    telemetry.update();
                }

                double leftPower;
                double rightPower;
                double centerPower;

                double controlCoeff = 1;

                if (gamepad1.a && aTimer.milliseconds() > 1000) {
                    fineControlMode = !fineControlMode;
                    aTimer.reset();
                }

                if (fineControlMode)
                    controlCoeff = 0.5;


                double drive = gamepad1.right_stick_y;
                double center = gamepad1.right_stick_x;
                leftPower = drive;
                rightPower = drive;
                centerPower = center;
                boolean turnLeft = gamepad1.left_bumper;
                boolean turnRight = gamepad1.right_bumper;
                if (turnLeft && !turnRight) {
                    leftPower = 0.5;
                    rightPower = -0.5;
                } else if (turnRight && !turnLeft) {
                    rightPower = 0.5;
                    leftPower = -0.5;
                }

                motor_left.setPower(leftPower * controlCoeff);
                motor_right.setPower(rightPower * controlCoeff);
                motor_center.setPower(centerPower * controlCoeff);

                //telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                //telemetry.addData("Fine Control Mode:", "" + fineControlMode);
                //telemetry.update();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.60;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }
}
