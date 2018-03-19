package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "test 1 servo", group = "Tutorials")


public class servo_test extends LinearOpMode {

    private ColorSensor color1;
    @Override
    public void runOpMode() throws InterruptedException {

        color1 = hardwareMap.colorSensor.get("color1");
        waitForStart();
        color1.enableLed(false);
        while (opModeIsActive()){
            telemetry.addData("ARGB:",color1.argb());
            telemetry.addData("RED:",color1.red());
            telemetry.addData("BLUE:",color1.blue());
            telemetry.addData("GREEN:",color1.green());
            telemetry.update();
            sleep(200);
        }
    }


}
