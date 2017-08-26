package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="WallShootBlueBeacon", group="Auto")
@Disabled






public class WallShootBeacon extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor intake;
    DcMotor lift;
    DcMotor shooter;
    ElapsedTime time;
    OpticalDistanceSensor LineSensor;
    ModernRoboticsI2cGyro gyro;
    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;


    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;


    static final double COUNTS_PER_MOTOR_REV = 1478.4;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    enum State {
        one, two, three, four, five, six, seven, eight, nine, ten,
        eleven, twelve, thirteen, fourteen, fifteen, sixteen, seventeen, eighteen, nineteen, twenty,
        twentyone, twentytwo, twentythree, twentyfour, twentyfive, twentysix,twentyseven,twentyeight,twentynine,thirty,thirtyone,thirtytwo,thirtythree,thrirtyfour,thirtyfive,thirtysix,thirtyseven,thirtyeight,thirtynine,forty
    }


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
        LineSensor = hardwareMap.opticalDistanceSensor.get("ods");
        colorC = hardwareMap.i2cDevice.get("cs");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        LineSensor.enableLed(true);
        colorC = hardwareMap.i2cDevice.get("cs");
        colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x1e), false);
        colorCreader.engage();

        time = new ElapsedTime();

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.calibrate();

        if (gyro.isCalibrating()) {
            state = State.one;
        }


        telemetry.update();


    }


    @Override
    public void loop() {
        double currentTime = time.time();
        double line = LineSensor.getLightDetected();
        readColor();
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        gyro.getHeading();


        switch (state) {

            case one:


                motorBackRight.setTargetPosition((int) (9 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-9 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (9 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-9 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(-.35);
                motorBackRight.setPower(.35);
                motorFrontLeft.setPower(-.35);
                motorFrontRight.setPower(.35);

                if (motorFrontRight.getCurrentPosition() >= (9 * COUNTS_PER_INCH)) {
                    state = state.two;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addLine("Case 1");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case two:

                motorBackRight.setTargetPosition((int) (-3 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-3 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (-3 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-3 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(-.75);
                motorBackRight.setPower(-.75);
                motorFrontLeft.setPower(-.75);
                motorFrontRight.setPower(-.75);

                if (motorFrontRight.getCurrentPosition() <= (-3 * COUNTS_PER_INCH)) {
                    state = state.three;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addLine("Case 2");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;

            case three:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setPower(-.2);
                motorBackRight.setPower(-.2);
                motorFrontLeft.setPower(-.2);
                motorFrontRight.setPower(-.2);


                if (gyro.getHeading() >= 40) {
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    state = state.four;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


                telemetry.addLine("Case 3");
                telemetry.addData("gyro", gyro.getHeading());
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;

            case four:

                motorBackRight.setTargetPosition((int) ( 28.137* COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-28.137 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (28.137 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-28.137 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(-.75);
                motorBackRight.setPower(.75);
                motorFrontLeft.setPower(-.75);
                motorFrontRight.setPower(.75);

                if (motorFrontRight.getCurrentPosition() >= (28.137 * COUNTS_PER_INCH)) {
                    state = state.five;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addLine("Case 4");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case five:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setPower(-.2);
                motorBackRight.setPower(-.2);
                motorFrontLeft.setPower(-.2);
                motorFrontRight.setPower(-.2);


                if (gyro.getHeading() >= 85) {
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    state = state.six;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


                telemetry.addLine("Case 5");
                telemetry.addData("gyro", gyro.getHeading());
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case six:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                motorBackLeft.setPower(-.3);
                motorBackRight.setPower(.3);
                motorFrontLeft.setPower(-.3);
                motorFrontRight.setPower(.3);

                if ((range1Cache[0] & 0xFF) <= 13) {
                    state = state.seven;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

                telemetry.addLine("Case 6");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case seven:

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorBackLeft.setPower(-.13);
                motorBackRight.setPower(-.13);
                motorFrontLeft.setPower(.13);
                motorFrontRight.setPower(.13);


                if (line >= 0.025 ){
                    state = state.eight;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }


                telemetry.addLine("Case 7");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;

            case eight:

                if (colorIsBlue()) {
                    state = state.nine;
                }

                if (colorIsRed()) {
                    state = state.ten;
                }


                telemetry.addLine("Case 8");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case nine:

                motorBackRight.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(-.13);
                motorBackRight.setPower(-.13);
                motorFrontLeft.setPower(.13);
                motorFrontRight.setPower(.13);

                if (motorFrontRight.getCurrentPosition()>= (2 * COUNTS_PER_INCH)) {
                    state = state.eleven;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addLine("Case 9");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case ten:


                motorBackRight.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(.13);
                motorBackRight.setPower(.13);
                motorFrontLeft.setPower(-.13);
                motorFrontRight.setPower(-.13);

                if (motorBackRight.getCurrentPosition()>=(2 * COUNTS_PER_INCH)) {
                    time.reset();
                    state=State.eleven;
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

                telemetry.addLine("Case 10");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;

            case eleven:


                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                motorBackLeft.setPower(-.2);
                motorBackRight.setPower(.2);
                motorFrontLeft.setPower(-.2);
                motorFrontRight.setPower(.2);

                if (currentTime >= 1) {
                    time.reset();
                    state=state.twelve;
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    telemetry.addLine("Case 11");
                    telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                    telemetry.addData("red", "red = " + colorIsRed());
                    telemetry.addData("blue", "blue = " + colorIsBlue());
                    telemetry.addData("Line", LineSensor.getLightDetected());
                    telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                    telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                    telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                    telemetry.addData("BR", motorBackRight.getCurrentPosition());
                    telemetry.update();


                }
                break;
            case twelve:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                motorBackLeft.setPower(.3);
                motorBackRight.setPower(-.3);
                motorFrontLeft.setPower(.3);
                motorFrontRight.setPower(-.3);

                if ((range1Cache[0] & 0xFF) >= 13){
                    state = state.thirteen;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

                telemetry.addLine("Case 12");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case thirteen:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setPower(-.2);
                motorBackRight.setPower(-.2);
                motorFrontLeft.setPower(-.2);
                motorFrontRight.setPower(-.2);


                if (gyro.getHeading() >= 85) {
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    state = state.fourteen;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


                telemetry.addLine("Case 13");
                telemetry.addData("gyro", gyro.getHeading());
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case fourteen:
            motorBackRight.setTargetPosition((int) (-25 * COUNTS_PER_INCH));
            motorBackLeft.setTargetPosition((int) (-25 * COUNTS_PER_INCH));
            motorFrontRight.setTargetPosition((int) (25 * COUNTS_PER_INCH));
            motorFrontLeft.setTargetPosition((int) (25* COUNTS_PER_INCH));
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBackLeft.setPower(-.35);
            motorBackRight.setPower(-.35);
            motorFrontLeft.setPower(.35);
            motorFrontRight.setPower(.35);

            if (motorFrontRight.getCurrentPosition()>=(25* COUNTS_PER_INCH)) {
                state = state.fifteen;
                time.reset();
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addLine("Case 14");
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("red", "red = " + colorIsRed());
            telemetry.addData("blue", "blue = " + colorIsBlue());
            telemetry.addData("Line", LineSensor.getLightDetected());
            telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
            telemetry.addData("FR", motorFrontRight.getCurrentPosition());
            telemetry.addData("BL", motorBackLeft.getCurrentPosition());
            telemetry.addData("BR", motorBackRight.getCurrentPosition());
            telemetry.update();

            break;

            case fifteen:

                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorBackLeft.setPower(-.13);
                motorBackRight.setPower(-.13);
                motorFrontLeft.setPower(.13);
                motorFrontRight.setPower(.13);


                if (line >= 0.025){
                    state = state.sixteen;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }


                telemetry.addLine("Case 15");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case sixteen:


                if (colorIsBlue()) {
                    state = state.seventeen;
                }

                if (colorIsRed()) {
                    state = state.eighteen;
                }


                telemetry.addLine("Case 16");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case seventeen:

                motorBackRight.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(-.13);
                motorBackRight.setPower(-.13);
                motorFrontLeft.setPower(.13);
                motorFrontRight.setPower(.13);

                if (motorFrontRight.getCurrentPosition()>=(2 * COUNTS_PER_INCH)) {
                    state = state.nineteen;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addLine("Case 17");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case eighteen:


                motorBackRight.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (2 * COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-2 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(.13);
                motorBackRight.setPower(.13);
                motorFrontLeft.setPower(-.13);
                motorFrontRight.setPower(-.13);

                if (motorBackRight.getCurrentPosition()>= (2 * COUNTS_PER_INCH)) {
                    time.reset();
                    state=State.nineteen;
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

                telemetry.addLine("Case 18");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;

            case nineteen:


                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                motorBackLeft.setPower(-.2);
                motorBackRight.setPower(.2);
                motorFrontLeft.setPower(-.2);
                motorFrontRight.setPower(.2);

                if (currentTime >= 1.5) {
                    time.reset();
                    state=state.twenty;
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    telemetry.addLine("Case 19");
                    telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                    telemetry.addData("red", "red = " + colorIsRed());
                    telemetry.addData("blue", "blue = " + colorIsBlue());
                    telemetry.addData("Line", LineSensor.getLightDetected());
                    telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                    telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                    telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                    telemetry.addData("BR", motorBackRight.getCurrentPosition());
                    telemetry.update();


                }
                break;
            case twenty:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                motorBackLeft.setPower(.5);
                motorBackRight.setPower(-.5);
                motorFrontLeft.setPower(.5);
                motorFrontRight.setPower(-.5);

                if ((range1Cache[0] & 0xFF) >= 88) {
                    state = state.twentyone;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

                telemetry.addLine("Case 20");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;

            case twentyone:
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setPower(-.35);
                motorBackRight.setPower(-.35);
                motorFrontLeft.setPower(-.35);
                motorFrontRight.setPower(-.35);


                if (gyro.getHeading() >=220 ) {
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    state = state.twentytwo;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


                telemetry.addLine("Case 21");
                telemetry.addData("gyro", gyro.getHeading());
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case twentytwo:
                motorBackRight.setTargetPosition((int) (25 * COUNTS_PER_INCH));
                motorBackLeft.setTargetPosition((int) (-25* COUNTS_PER_INCH));
                motorFrontRight.setTargetPosition((int) (25 * COUNTS_PER_INCH));
                motorFrontLeft.setTargetPosition((int) (-25 * COUNTS_PER_INCH));
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackLeft.setPower(-1);
                motorBackRight.setPower(1);
                motorFrontLeft.setPower(-1);
                motorFrontRight.setPower(1);

                if (motorFrontRight.getCurrentPosition() >= (25 * COUNTS_PER_INCH)) {
                    state = state.twentythree;
                    time.reset();
                    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addLine("Case 22");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();

                break;
            case twentythree:
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);

                telemetry.addLine("Case 23");
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.addData("red", "red = " + colorIsRed());
                telemetry.addData("blue", "blue = " + colorIsBlue());
                telemetry.addData("Line", LineSensor.getLightDetected());
                telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
                telemetry.addData("FR", motorFrontRight.getCurrentPosition());
                telemetry.addData("BL", motorBackLeft.getCurrentPosition());
                telemetry.addData("BR", motorBackRight.getCurrentPosition());
                telemetry.update();
                break;



        }
    }


    public void readColor() {
        colorCcache = colorCreader.read(0x04, 1);
    }

    boolean colorIsRed() {
        if ((colorCcache[0] & 0xFF) == 10 ||
                (colorCcache[0] & 0xFF) == 11) {
            return (true);
        } else return (false);
    }

    boolean colorIsBlue() {
        if ((colorCcache[0] & 0xFF) == 2 ||
                (colorCcache[0] & 0xFF) == 3) {
            return (true);
        } else return (false);
    }
}

