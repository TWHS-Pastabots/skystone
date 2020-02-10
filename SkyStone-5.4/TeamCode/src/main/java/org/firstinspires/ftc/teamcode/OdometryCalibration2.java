package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OdometryCalibration2 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime= new ElapsedTime();



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


    }

    public double getAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }
}
