package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AtomicGears", group = "test")

//This is our main teleop drive code

public class AtomicGears extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor intake;
    DcMotor lift;
    DcMotor shooter;
    DcMotor cap;
    DcMotor led;
    OpticalDistanceSensor LineSensor;




    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackLeft = hardwareMap.dcMotor.get("bl");
        motorBackRight = hardwareMap.dcMotor.get("br");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");
        shooter = hardwareMap.dcMotor.get("shoot");
        cap = hardwareMap.dcMotor.get("cap");
        LineSensor = hardwareMap.opticalDistanceSensor.get("ods");
        led = hardwareMap.dcMotor.get("led");


    }

    @Override
    public void loop() {


        //this is the primary driver buttons and functions

        if(gamepad1.left_bumper)  {intake.setPower(1);}
        else if(gamepad1.right_bumper){intake.setPower(-1);}
        else {intake.setPower(0);}

        if(gamepad1.x)  {lift.setPower(1);}
        else if(gamepad1.y){lift.setPower(-1);}
        else {lift.setPower(0);}

        if(gamepad1.a)  {shooter.setPower(1);}
        else if(gamepad1.b){shooter.setPower(-1);}
        else {shooter.setPower(0);}

        //this is the second drivers button to control the lights

        if(gamepad2.a) {
            led.setPower(1);
        }
        if (gamepad2.b){
            led.setPower(0);
        }

        /* right stick X controls rotation */

        float gamepad1LeftY = -gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad1RightX = gamepad1.right_stick_x;
        float gamepad2RightY = gamepad2.right_stick_y;

        // holonomic formulas

        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);
        gamepad2RightY = Range.clip(gamepad2RightY, -1,1);

        FrontLeft = (float) scaleInput(FrontLeft);
        FrontRight = (float) scaleInput(FrontRight);
        BackLeft = (float) scaleInput(BackLeft);
        BackRight =  (float) scaleInput(BackRight);
        gamepad2RightY= (float) scaleInput(gamepad2RightY);

        // write the values to the motors
        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);
        cap.setPower(gamepad2RightY);

		/*
		 * Telemetry for debugging
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Joy XL YL XR",  String.format("%.2f", gamepad1LeftX) + " " +
                String.format("%.2f", gamepad1LeftY) + " " +  String.format("%.2f", gamepad1RightX));
        telemetry.addData("f left pwr",  "front left  pwr: " + String.format("%.2f", FrontLeft));
        telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
        telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
        telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
        telemetry.addData("Intake",intake.getPower());
        telemetry.addData("Lift",lift.getPower());
        telemetry.addData("Shooter",shooter.getPower());
        telemetry.addData("Line", LineSensor.getLightDetected());
    }

    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds. We borrowed this from team 4962 Rockettes github.
     * thank you for sharing!
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}