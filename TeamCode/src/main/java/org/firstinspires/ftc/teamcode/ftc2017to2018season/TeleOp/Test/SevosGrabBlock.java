package org.firstinspires.ftc.teamcode.ftc2017to2018season.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Inspiration Team on 4/8/2018.
 */
@TeleOp(name = "Servo Grab Block")
@Disabled
public class SevosGrabBlock extends OpMode {

    Servo leftServo;
    Servo rightServo;
    Gamepad gamepad = new Gamepad();


    public void init(){
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");

        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void loop(){
        //testServo.setPosition(0);
    }

}
