package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;



@TeleOp(name ="Range", group = "Sensor")
@Disabled

public class Range extends LinearOpMode {


    /*
     * Main loop
     */
    @Override
    public void runOpMode() throws InterruptedException {

        AnalogInput distanceSensor;
        distanceSensor = hardwareMap.analogInput.get("distance");


        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
           distanceSensor.getVoltage();

            final double MeasuredVoltage = distanceSensor.getVoltage();
            final double VoltageScale = 0.009766;
            final double RangeInInches = MeasuredVoltage/VoltageScale;


                    telemetry.addData("inches",RangeInInches);
            telemetry.addData("volt",distanceSensor.getVoltage());
            telemetry.update();
        }
    }


}