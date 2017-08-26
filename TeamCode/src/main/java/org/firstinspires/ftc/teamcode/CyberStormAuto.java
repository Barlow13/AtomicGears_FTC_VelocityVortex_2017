package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Owner on 9/17/2016.
 */

@Autonomous(name="CapBall", group="Test")
@Disabled


public class CyberStormAuto extends OpMode {

    DcMotor Left;
    DcMotor Right;
    DcMotor Intake;
    DcMotor Shoot1;
    DcMotor Shoot2;
    DcMotor lift;
    Servo cap;
    Servo ball;
    ElapsedTime time;

    static final double COUNTS_PER_MOTOR_REV = 1478.4;    // eg: Matrix Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    enum State {
        one, two, three, four, five, six, seven, eight, nine, ten
    }

    State state;


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
        time = new ElapsedTime();
                  Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                  Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        state = State.one;

    }

    @Override
    public void loop() {
        double currentTime = time.time();
        switch (state) {

            case one:
                Left.setTargetPosition((int) (COUNTS_PER_INCH * 65));
                Right.setTargetPosition((int) (COUNTS_PER_INCH * -65));
                Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Left.setPower(.75);
                Right.setPower(-.75);

                telemetry.addLine("Case 1");
                telemetry.addData("Left", Left.getCurrentPosition());
                telemetry.addData("Right", Right.getCurrentPosition());

                if (Left.getCurrentPosition() >= (COUNTS_PER_INCH * 65)) {
                    Left.setPower(0);
                    Right.setPower(0);
                    Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    state = state.two;

                }
                break;
            case two:
                Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Left.setPower(0);
                Right.setPower(0);

                telemetry.addLine("Case 1");
                telemetry.addData("Left", Left.getCurrentPosition());
                telemetry.addData("Right", Right.getCurrentPosition());
                break;




        }
    }
}