package org.firstinspires.ftc.teamcode;

import android.widget.ToggleButton;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import android.view.View;

@Autonomous(name = "Dreapta rosu", group="test")
public class auto_mode_red extends LinearVisionOpMode {
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorCatapulta;
    private Servo servo1;
    private OpticalDistanceSensor optical1;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private GyroSensor gyro;
    public boolean teamColor=true;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorCatapulta = hardwareMap.dcMotor.get("motorCatapulta");
        servo1=hardwareMap.servo.get("servo1");
        optical1=hardwareMap.opticalDistanceSensor.get("optical1");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        gyro= hardwareMap.gyroSensor.get("gyro");

    //FTC Vision
        gyro.calibrate();
        waitForVisionStart();
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        //END FTC Vision
        //Servo reset
        ElapsedTime mRuntime = new ElapsedTime();

        servo1.setPosition(0.5);

        waitForStart();

        if (opModeIsActive()) {
            while(rangeSensor.getDistance(DistanceUnit.CM)<64) //mers la stanga pana la cos
            {
                change_motor_power(0.15,0.15,-0.15,-0.15);
                telemetry.addData("Distanta actuala",rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            change_motor_power(0,0,0,0);
            mRuntime.reset();

            while(mRuntime.time()<3.5)
                motorCatapulta.setPower(1);//dat la cos kobe

            motorCatapulta.setPower(0);

            while (gyro.getHeading()<90)//roteste stanga
            {
                change_motor_power(0.1,0.1,0.1,0.1);
            }
            change_motor_power(0,0,0,0);
            telemetry.addData("distanta",rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            while(rangeSensor.getDistance(DistanceUnit.CM)>35)//mers dreapta pana la perete
            {
                telemetry.addData("distanta",rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
                change_motor_power(-0.3,-0.2,0.2,0.3);
                if(gyro.getHeading()<85 || gyro.getHeading()>95) {
                    while(gyro.getHeading()<85 || gyro.getHeading()>95)
                    {
                        if(gyro.getHeading()>90)
                        {
                            while(gyro.getHeading()<85||gyro.getHeading()>95)
                            {
                                change_motor_power(-0.05,-0.05,-0.05,-0.05);
                                telemetry.addData("Gyro:",gyro.getHeading());
                                telemetry.update();
                            }
                            telemetry.addData("Gyro:",gyro.getHeading());
                            telemetry.update();
                        }
                        if(gyro.getHeading()<90)
                        {
                            while(gyro.getHeading()<85 || gyro.getHeading()>95)
                            {
                                change_motor_power(0.05,0.05,0.05,0.05);
                                telemetry.addData("Gyro:",gyro.getHeading());
                                telemetry.update();
                            }
                            telemetry.addData("Gyro:",gyro.getHeading());
                            telemetry.update();
                        }
                    }
                }

            }
            change_motor_power(0,0,0,0);
            while(optical1.getLightDetected()<0.02)
                change_motor_power(-0.1,0.1,-0.1,0.1);
            change_motor_power(0,0,0,0);
            telemetry.addData("beacon colors:",beacon.getAnalysis().getColorString());
            telemetry.update();
            if(beacon.getAnalysis().getColorString().charAt(3)==','){
                servo1.setPosition(0);
            }
            else {
                servo1.setPosition(1);
            }


            mRuntime.reset();
            while(mRuntime.time()<1)
                change_motor_power(-0.2,-0.2,0.2,0.2);//mers dreapta si apasa pe beacon
            change_motor_power(0,0,0,0);

            mRuntime.reset();
            while(mRuntime.time()<1)//merge oleaca inainte,sa iasa de pe linia alba
                change_motor_power(-0.1,0.1,-0.1,0.1);
            change_motor_power(0,0,0,0);

            servo1.setPosition(0.5);
            telemetry.addData("detected light",optical1.getLightDetected());
            telemetry.update();
            while(optical1.getLightDetected()<0.02) {
                change_motor_power(-0.1, 0.1, -0.1, 0.1);
                if (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                    while (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                        if (gyro.getHeading() > 90) {
                            while (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                                change_motor_power(-0.05, -0.05, -0.05, -0.05);
                                telemetry.addData("Gyro:", gyro.getHeading());
                                telemetry.update();
                            }
                            telemetry.addData("Gyro:", gyro.getHeading());
                            telemetry.update();
                        }
                        if (gyro.getHeading() < 90) {
                            while (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                                change_motor_power(0.05, 0.05, 0.05, 0.05);
                                telemetry.addData("Gyro:", gyro.getHeading());
                                telemetry.update();
                            }
                            telemetry.addData("Gyro:", gyro.getHeading());
                            telemetry.update();
                        }
                    }
                }
            }
            change_motor_power(0,0,0,0);

            telemetry.addData("beacon colors:",beacon.getAnalysis().getColorString());
            telemetry.update();
            if(beacon.getAnalysis().getColorString().charAt(3)==','){
                servo1.setPosition(0);
            }
            else {
                servo1.setPosition(1);
            }


            mRuntime.reset();
            while(mRuntime.time()<1)
                change_motor_power(-0.2,-0.2,0.2,0.2);//mers dreapta si apasa pe beacon
            change_motor_power(0,0,0,0);

            mRuntime.reset();
            while(mRuntime.time()<2)
                change_motor_power(1,0,0,-1);
        }

    }

    private void change_motor_power(double motorFrontLeftPow,double motorFrontRightPow,double motorBackLeftPow,double motorBackRightPow){
        motorFrontLeft.setPower(motorFrontLeftPow);
        motorFrontRight.setPower(motorFrontRightPow);

        motorBackLeft.setPower(motorBackLeftPow);
        motorBackRight.setPower(motorBackRightPow);
    }

}