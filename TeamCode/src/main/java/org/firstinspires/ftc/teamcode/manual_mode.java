package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "test 1 motor", group = "Tutorials")
public class manual_mode extends LinearOpMode {
    // Declare drive motors
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorRotatie1;
    private DcMotor motorRotatie2;
/*
    private DcMotor motorCatapulta;
*/
    private Servo servo1;

    public int left_stick_atrest()
    {
        if(gamepad1.left_stick_x==0&&gamepad1.left_stick_y==0)
            return 0;
        else return 1;
    }
    public int right_stick_atrest()
    {
        if(gamepad1.right_stick_x==0&&gamepad1.right_stick_y==0)
            return 0;
        else return 1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorRotatie1 = hardwareMap.dcMotor.get("motorRotatie1");
        motorRotatie2 = hardwareMap.dcMotor.get("motorRotatie2");
/*
        motorCatapulta = hardwareMap.dcMotor.get("motorCatapulta");
*/

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // Wait until start button is pressed
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        while (opModeIsActive()) {

            if(left_stick_atrest()==0&&right_stick_atrest()==1 ) {
                if (gamepad1.right_stick_y < 0 && gamepad1.right_stick_x > 0)
                {
                    motorFrontLeft.setPower(gamepad1.right_stick_y);
                    motorBackRight.setPower(-gamepad1.right_stick_y);
                }
                if (gamepad1.right_stick_y < 0 && gamepad1.right_stick_x < 0)
                {
                    motorFrontRight.setPower(-gamepad1.right_stick_y);
                    motorBackLeft.setPower(gamepad1.right_stick_y);
                }
                if (gamepad1.right_stick_y > 0 && gamepad1.right_stick_x < 0)
                {
                    motorFrontLeft.setPower(gamepad1.right_stick_y);
                    motorBackRight.setPower(-gamepad1.right_stick_y);
                }
                if (gamepad1.right_stick_y > 0 && gamepad1.right_stick_x > 0)
                {
                    motorFrontRight.setPower(-gamepad1.right_stick_y);
                    motorBackLeft.setPower(gamepad1.right_stick_y);
                }
            }

            telemetry.addData("Left Stick x: ",gamepad1.left_stick_x);
            telemetry.addData("Left Stick y: ",gamepad1.left_stick_y);
            telemetry.addData("Left Bumper: ",gamepad1.left_bumper);
            telemetry.addData("Right bumper: ",gamepad1.right_bumper);
            telemetry.addData("Right stick x: ",gamepad1.right_stick_x);
            telemetry.addData("Right stick y: ",gamepad1.right_stick_y);
            telemetry.addData("Left Trigger: ",gamepad1.left_trigger);
            telemetry.addData("Right Trigger: ",gamepad1.right_trigger);
            telemetry.update();

            if (gamepad1.a) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);

                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }

            if(left_stick_atrest()==1&&right_stick_atrest()==0) {
                if (gamepad1.left_stick_x > -0.4 && gamepad1.left_stick_x < 0.4 ) {
                            motorFrontLeft.setPower(gamepad1.left_stick_y);
                            motorBackLeft.setPower(gamepad1.left_stick_y);

                            motorFrontRight.setPower(-gamepad1.left_stick_y);
                            motorBackRight.setPower(-gamepad1.left_stick_y);
                }
                else  {
                    motorFrontLeft.setPower(-gamepad1.left_stick_x);
                    motorBackLeft.setPower(gamepad1.left_stick_x);

                    motorFrontRight.setPower(-gamepad1.left_stick_x);
                    motorBackRight.setPower(gamepad1.left_stick_x);

                }
            }
            if(gamepad1.left_trigger>0)
            {
                motorBackRight.setPower(gamepad1.left_trigger);
                motorBackLeft.setPower(gamepad1.left_trigger);
                motorFrontRight.setPower(gamepad1.left_trigger);
                motorFrontLeft.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger>0)
            {
                motorBackRight.setPower(-gamepad1.right_trigger);
                motorBackLeft.setPower(-gamepad1.right_trigger);
                motorFrontRight.setPower(-gamepad1.right_trigger);
                motorFrontLeft.setPower(-gamepad1.right_trigger);
            }

            if(gamepad1.atRest())
            {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);

                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }

            if(gamepad1.left_bumper)
            {
                motorRotatie1.setPower(1);
                motorRotatie2.setPower(1);
            }
            if (gamepad1.right_bumper)
            {
                motorRotatie1.setPower(-1);
                motorRotatie2.setPower(-1);
            }
            if(gamepad1.y)
            {
                motorRotatie1.setPower(0);
                motorRotatie2.setPower(0);
            }
/*            if(gamepad1.b)
            {
                motorCatapulta.setPower(1);
            }
            else
            {
                motorCatapulta.setPower(0);
            }*/


        }
    }
}


