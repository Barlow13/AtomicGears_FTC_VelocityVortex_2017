package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto", group="Auto")
@Disabled




public class auto extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor intake;
    DcMotor lift;
    DcMotor shooter;
    ElapsedTime time;
    static final double half = 0.5;
    static final double one = 1.0;
    static final double two = 2.0;
    static final double three = 3.0;
    static final double five = 5.0;
    static final double six = 6.0;

    static final double COUNTS_PER_MOTOR_REV = 1478.4;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    enum State {DrivingStraight, Shoot, Load, Shoot2, HitBall, MoveFromWall, Correct}

    State state;

    @Override
    public void init() {


        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackLeft = hardwareMap.dcMotor.get("bl");
        motorBackRight = hardwareMap.dcMotor.get("br");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");
        shooter = hardwareMap.dcMotor.get("shoot");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           time = new ElapsedTime();
        state = State.MoveFromWall;


    }

    @Override
    public void loop() {
        double currentTime = time.time();

        switch (state) {
            case MoveFromWall:
                motorBackRight.setTargetPosition((int) (2.896 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-2.896* COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (2.896* COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-2.869* COUNTS_PER_INCH));

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                motorBackLeft.setPower(-1);
                motorBackRight.setPower(1);
                motorFrontLeft.setPower(-1);
                motorFrontRight.setPower(1);

                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());

                telemetry.update();


                if (currentTime >= three ) {
                    state = state.Shoot;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case Shoot:
                shooter.setPower(1);

                if (currentTime >= one){
                    state = state.Load;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setPower(0);

                }


                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());

                telemetry.update();
                break;

            case Load:
                lift.setPower(-1);

                if (currentTime >= 2){
                state = state.Shoot2;
                time.reset();
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setPower(0);
                 }
                break;

            case Shoot2:
                shooter.setPower(1);

                if (currentTime >= one){
                    state = State.DrivingStraight;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setPower(0);
                }


                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());

                telemetry.update();
                break;
            case DrivingStraight:
                motorBackRight.setTargetPosition((int) (48* COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-48* COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (48* COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-48* COUNTS_PER_INCH));

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                motorBackLeft.setPower(-.50);
                motorBackRight.setPower(.50);
                motorFrontLeft.setPower(-.50);
                motorFrontRight.setPower(.50);

                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());

                telemetry.update();
                if (currentTime >= 5){
                    state = state.HitBall;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;

        }

    }

}

