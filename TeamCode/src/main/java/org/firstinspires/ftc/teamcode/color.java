package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Brady on 11/23/2016.
 */
@TeleOp (name = "color", group = "Sensor")
@Disabled
public class color extends OpMode {


    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;



    @Override
    public void init() {

        colorC = hardwareMap.i2cDevice.get("cs");
        colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x1e), false);
        colorCreader.engage();


    }

    @Override
    public void loop() {
        readColor();
        telemetry.addData("red","red = " + colorIsRed());
        telemetry.addData("blue","blue = " + colorIsBlue());
        telemetry.update();

    }
        public void readColor(){
            colorCcache = colorCreader.read(0x04, 1);
        }

        boolean colorIsRed() {
            if ((colorCcache[0] & 0xFF) == 10 ||
                    (colorCcache[0] & 0xFF) == 11) {
                return (true);
            }
            else return (false);
        }
        boolean colorIsBlue() {
            if ((colorCcache[0] & 0xFF) == 2 ||
                    (colorCcache[0] & 0xFF) == 3) {
                return (true);
            } else return (false);
        }

}
