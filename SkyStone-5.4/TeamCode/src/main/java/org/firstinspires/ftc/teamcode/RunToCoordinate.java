package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.reference_code.AutoSkystoneGyroBlue;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Beginning of a RunToCoordinate class that can use information from
 *the Positioning thread and RigatoniOdometryCalibration in order to send powers to the
 * encoders to get the bot from position A to position B.
 */

@Autonomous
public class RunToCoordinate extends LinearOpMode {

    private BNO055IMU imu;
    private DistanceSensor color;
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera phoneCam;
    private RunToCoordinate.DetectorPipeline detectorPipeline;
    private String stoneConfig;

    private RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        detectorPipeline = new RunToCoordinate.DetectorPipeline();
        phoneCam.setPipeline(detectorPipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);


        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        Positioning positioning = new Positioning(robot.leftEnc, robot.rightEnc, robot.horizEnc, 50, robot.imu, 0.0, 0, 0);
        Thread positionThread = new Thread(positioning);
        positionThread.start();

        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", positioning.getX());
            telemetry.addData("Y Position", positioning.getY());
            telemetry.addData("Orientation (Degrees)", positioning.getOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();


        }

        positioning.stop();
    }

    class DetectorPipeline extends OpenCvPipeline {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat mat0 = new Mat();
        private Mat mat1 = new Mat();
        private Mat mat2 = new Mat();

        private boolean madeMats = false;

        private Mat mask0;
        private Mat mask1;
        private Mat mask2;


        private final Scalar BLACK = new Scalar(0, 0, 0);
        private final Scalar WHITE = new Scalar(256, 256, 256);
        private final int r = 10;
        private final int cx0 = 75, cx1 = 140, cx2 = 220;// Width=320 Height=240
        private final int cy0 = 175, cy1 = 175, cy2 = 175;

        private String detectedPos;

        @Override
        public Mat processFrame(Mat input) {
            int h = input.height();
            int w = input.width();
            int t = input.type();
            if (!madeMats) {
                mask0 = new Mat(h, w, t);
                mask1 = new Mat(h, w, t);
                mask2 = new Mat(h, w, t);
                madeMats = true;
            }

            mask0.setTo(BLACK);
            mask1.setTo(BLACK);
            mask2.setTo(BLACK);

            Imgproc.circle(mask0, new Point(cx0, cy0), r, WHITE, Core.FILLED);
            Imgproc.circle(mask1, new Point(cx1, cy1), r, WHITE, Core.FILLED);
            Imgproc.circle(mask2, new Point(cx2, cy2), r, WHITE, Core.FILLED);

            Core.bitwise_and(mask0, input, mat0);
            Core.bitwise_and(mask1, input, mat1);
            Core.bitwise_and(mask2, input, mat2);


            double sum0 = sum(Core.sumElems(mat0).val);
            double sum1 = sum(Core.sumElems(mat1).val);
            double sum2 = sum(Core.sumElems(mat2).val);

            if (sum0 < sum1 && sum0 < sum2)
                detectedPos = "Left";
            else if (sum1 < sum0 && sum1 < sum2)
                detectedPos = "Middle";
            else if (sum2 < sum1 && sum2 < sum0)
                detectedPos = "Right";


            /*
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            Imgproc.circle(input, new Point(cx0, cy0), r, WHITE, Core.FILLED);
            Imgproc.circle(input, new Point(cx1, cy1), r, WHITE, Core.FILLED);
            Imgproc.circle(input, new Point(cx2, cy2), r, WHITE, Core.FILLED);

            return input;
        }

        public double sum(double[] arr) {
            double sum = 0.0;
            for (int i = 0; i < arr.length; i++) {
                sum += arr[i];
            }
            return sum;
        }

        public String getDetectedPos() {
            return detectedPos;
        }
    }
}
