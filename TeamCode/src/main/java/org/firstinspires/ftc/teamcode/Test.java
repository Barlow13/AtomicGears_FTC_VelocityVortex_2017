package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Owner on 9/17/2016.
 */

@TeleOp(name="CyberStorm", group="Test")
@Disabled


public class Test extends OpMode {

    DcMotor Left;
    DcMotor Right;
    DcMotor Intake;
    DcMotor Shoot1;
    DcMotor Shoot2;
    DcMotor lift;
    Servo cap;
    Servo ball;


    @Override
    public void init() {

        Left = hardwareMap.dcMotor.get("L");
        Right = hardwareMap.dcMotor.get("R");
        Intake = hardwareMap.dcMotor.get("I");
        Shoot1 = hardwareMap.dcMotor.get("s1");
        Shoot2 = hardwareMap.dcMotor.get("s2");
        lift = hardwareMap.dcMotor.get("lift");
        ball = hardwareMap.servo.get("ball");
        cap = hardwareMap.servo.get("cap");
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            Intake.setPower(-1);
        } else if (gamepad1.right_bumper) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
        if (gamepad1.a) {
            ball.setPosition(1);
        } else {
            ball.setPosition(0);
        }

        if (gamepad2.a) {
            cap.setPosition(1);
        }
        else {cap.setPosition(0);}


        float shoot = gamepad1.left_trigger;
        float capball = -gamepad2.right_stick_y;

        float left  = gamepad1.left_stick_y - gamepad1.right_stick_x;
        float right = gamepad1.left_stick_y +gamepad1.right_stick_x;

        left = (float) scaleInput(left);
        right = (float) scaleInput(right);
        Left.setPower(left);
        Right.setPower(-right);
        Shoot1.setPower(shoot);
        Shoot2.setPower(-shoot);
        lift.setPower(capball);

    }

    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
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

