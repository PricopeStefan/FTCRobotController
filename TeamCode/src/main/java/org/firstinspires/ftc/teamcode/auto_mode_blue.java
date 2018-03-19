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

@Autonomous(name = "Dreapta albastru", group="test")
public class auto_mode_blue extends LinearVisionOpMode {
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
    private double DistantaInitiala;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorCatapulta = hardwareMap.dcMotor.get("motorCatapulta");
        servo1=hardwareMap.servo.get("servo1");
        optical1=hardwareMap.opticalDistanceSensor.get("optical1");
        gyro=hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        gyro.calibrate();
        //FTC Vision
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
        DistantaInitiala=rangeSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distanta initiala",DistantaInitiala);
        telemetry.update();
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

            while(gyro.getHeading()<82 || gyro.getHeading()>98)// rotire stanga
                change_motor_power(0.1,0.1,0.1,0.1);
            change_motor_power(0,0,0,0);
            telemetry.addData("distanta",rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            while(rangeSensor.getDistance(DistanceUnit.CM)>35)//mers dreapta pana la perete
            {
                telemetry.addData("distanta",rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
                change_motor_power(-0.2,-0.12,0.12,0.2);
                if(gyro.getHeading()<85 || gyro.getHeading()>95) {
                    while(gyro.getHeading()<85 || gyro.getHeading()>95)
                    {
                        if(gyro.getHeading()>90)
                        {
                            while(gyro.getHeading()<85||gyro.getHeading()>95)
                            {
                                change_motor_power(-0.23,-0.15,0.12,0.2);
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
                                change_motor_power(-0.2,-0.12,0.15,0.23);
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
                servo1.setPosition(1);
            }
            else {
                servo1.setPosition(0);
            }


            mRuntime.reset();
            while(mRuntime.time()<1.5)
                change_motor_power(-0.2,-0.2,0.2,0.2);//mers dreapta si apasa pe beacon
            change_motor_power(0,0,0,0);

            mRuntime.reset();
            while(mRuntime.time()<1)//merge oleaca inainte,sa iasa de pe linia alba
                change_motor_power(-0.2,0.2,-0.2,0.2);
            change_motor_power(0,0,0,0);

            servo1.setPosition(0.5);
            telemetry.addData("detected light",optical1.getLightDetected());
            telemetry.update();
            /*while(optical1.getLightDetected()<0.02) {
                change_motor_power(-0.15, 0.15, -0.15, 0.15);
                if (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                    while (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                        if (gyro.getHeading() > 90) {
                            while (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                                change_motor_power(-0.175, 0.15, -0.175, 0.15);
                                telemetry.addData("Gyro:", gyro.getHeading());
                                telemetry.update();
                            }
                            telemetry.addData("Gyro:", gyro.getHeading());
                            telemetry.update();
                        }
                        if (gyro.getHeading() < 90) {
                            while (gyro.getHeading() < 85 || gyro.getHeading() > 95) {
                                change_motor_power(-0.15, 0.175, -0.15, 0.175);
                                telemetry.addData("Gyro:", gyro.getHeading());
                                telemetry.update();
                            }
                            telemetry.addData("Gyro:", gyro.getHeading());
                            telemetry.update();
                        }
                    }
                }
            }*/
                if(rangeSensor.getDistance(DistanceUnit.CM)!=20) {//redresarea distanta
                    while (rangeSensor.getDistance(DistanceUnit.CM) < 20)
                        change_motor_power(0.1, 0.1, -0.1, -0.1);
                    while(rangeSensor.getDistance(DistanceUnit.CM)>20)
                        change_motor_power(-0.1,-0.1,0.1,0.1);
                }
            if(gyro.getHeading()!=90) { //redresare gyro
                if (gyro.getHeading() < 90)
                    while (gyro.getHeading()!=90)
                        change_motor_power(0.07, 0.07, 0.07, 0.07);
                if(gyro.getHeading() > 90)
                    while(gyro.getHeading()!=90)
                        change_motor_power(-0.07,-0.07,-0.07,-0.07);
            }
            change_motor_power(0,0,0,0);
            while(optical1.getLightDetected()<0.02)
                change_motor_power(-0.15,0.15,-0.15,0.15);
            change_motor_power(0,0,0,0);
            telemetry.addData("beacon colors:",beacon.getAnalysis().getColorString());
            telemetry.update();
            if(beacon.getAnalysis().getColorString().charAt(3)==','){
                servo1.setPosition(1);
            }
            else {
                servo1.setPosition(0);
            }


            mRuntime.reset();
            while(mRuntime.time()<1.5)
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