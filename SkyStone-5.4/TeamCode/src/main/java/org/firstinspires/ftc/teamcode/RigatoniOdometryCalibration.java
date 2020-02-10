package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;

/**
 * Created by Cam and Matthew on 02092020.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Positioning Algorithm will not function and will throw an error if this program is not run first
 */

@TeleOp
public class RigatoniOdometryCalibration extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime= new ElapsedTime();
    static final double POWER = 0.3;
    static final int TRIALS = 3;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File verticalLeftTickOffsetFile = AppUtil.getInstance().getSettingsFile("verticalLeftTickOffset.txt");
    File verticalRightTickOffsetFile = AppUtil.getInstance().getSettingsFile("verticalRightTickOffset.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        robot.leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        robot.leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.horizEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!isStarted()) {
            telemetry.addData("Odometry System Calibration Status", "Init Complete");
            telemetry.addData("IMU Angle", getAngle());
            telemetry.addData("Vertical Left Position", robot.leftEnc.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.rightEnc.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        double sumDiffLeft = 0.0;
        double sumDiffRight = 0.0;
        double sumDiffHoriz = 0.0;

        for(int i = 0; i < TRIALS; i++){

            double startPosLeft = robot.leftEnc.getCurrentPosition();
            double startPosRight = robot.rightEnc.getCurrentPosition();
            double startPosHoriz = robot.horizEnc.getCurrentPosition();
            double startAngle = getAngle();

            runTime.reset();
            while(runTime.milliseconds() < 1000){
                robot.leftFront.setPower(-POWER);
                robot.rightFront.setPower(POWER);
                robot.leftRear.setPower(-POWER);
                robot.rightRear.setPower(POWER);
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftRear.setPower(0);
            robot.rightRear.setPower(0);
            sleep(1000);

            sumDiffLeft += (robot.leftEnc.getCurrentPosition() - startPosLeft) / (getAngle() - startAngle);
            sumDiffRight += (robot.rightEnc.getCurrentPosition() - startPosRight) / (getAngle() - startAngle);
            sumDiffHoriz += (robot.horizEnc.getCurrentPosition() - startPosHoriz) / (getAngle() - startAngle);
        }

        double verticalLeftEncoderTickOffsetPerDegree = sumDiffLeft / (double)TRIALS;
        double verticalRightEncoderTickOffsetPerDegree = sumDiffRight / (double)TRIALS;
        double horizontalEncoderTickOffsetPerDegree = sumDiffHoriz / (double)TRIALS;

        //Write the constants to text files
        ReadWriteFile.writeFile(verticalLeftTickOffsetFile, String.valueOf(verticalLeftEncoderTickOffsetPerDegree));
        ReadWriteFile.writeFile(verticalRightTickOffsetFile, String.valueOf(verticalRightEncoderTickOffsetPerDegree));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalEncoderTickOffsetPerDegree));

        while(opModeIsActive()){
            telemetry.addData("verticalLeftEncoderTickOffsetPerDegree", ""+verticalLeftEncoderTickOffsetPerDegree);
            telemetry.addData("verticalRightEncoderTickOffsetPerDegree", ""+verticalRightEncoderTickOffsetPerDegree);
            telemetry.addData("horizontalEncoderTickOffsetPerDegree", ""+horizontalEncoderTickOffsetPerDegree);
            telemetry.update();
        }

    }

    public double getAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle < 0 ?
                -robot.imu.getAngularOrientation().firstAngle + 360 :
                -robot.imu.getAngularOrientation().firstAngle);
    }
}
