package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name ="touch", group = "Sensor")
@Disabled

public class touch extends LinearOpMode {


    /*
     * Main loop
     */
    @Override
    public void runOpMode() throws InterruptedException {

        TouchSensor touch;
        touch = hardwareMap.touchSensor.get("touch");


        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
           if (touch.isPressed())
               telemetry.addLine("true");
            else telemetry.addLine("false");
            touch.getValue();
            telemetry.addData("value", touch.getValue());

            telemetry.update();
        }
    }


}