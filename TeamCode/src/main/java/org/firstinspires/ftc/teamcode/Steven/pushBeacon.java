package org.firstinspires.ftc.teamcode.Steven;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by inspirationteam on 12/2/2016.
 */
@Autonomous(name = "pushBeacon", group = "Pushbot")
public class pushBeacon extends OpMode {
    Servo leftPlate;
    Servo rightPlate;
    boolean bLedOn;

    ColorSensor colorSensor;

    @Override
    public void init(){
        leftPlate = hardwareMap.servo.get("leftplate");
        rightPlate = hardwareMap.servo.get("rightplate");

        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        bLedOn = false;
        colorSensor.enableLed(bLedOn);
    }

    @Override
    public void init_loop(){


    }

    @Override
    public void start() {//main autonomous mode goes in here

        //wait 20 seconds

        if(ReadColorSensor() == 'r'){
            //push button if on blue team, don't push if on red team
        }
        else if (ReadColorSensor() == 'b'){
            //push button if on red team, don't push if on blue team
        }
    }
    public void loop(){

    }

    public void stop(){

    }


    public char ReadColorSensor(){

        char redOrBlue = 'b';

    /* Variables used to store value of the color sensor*/
    /* hsvValues is an array that will hold the hue, saturation, and value information */
        float hsvValues[] = {0F,0F,0F};



    /* convert the RGB values to HSV values*/
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

    /* send the info back to driver station using telemetry function.*/
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);


        if (colorSensor.red()>colorSensor.blue()) {/*need to find out the treashold for */
            redOrBlue = 'r';
        }
        telemetry.addData("Red or Blue", redOrBlue);
        telemetry.update();
        return redOrBlue;
    }

}
