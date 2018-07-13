package org.firstinspires.ftc.teamcode.ftc2017to2018season.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Rohan on 11/18/17.
 */


@TeleOp(name = "servo read position")
@Disabled
public class servo_valueRead extends OpMode {


    Servo glyphServoRight;
    Servo glyphServoLeft;
    Servo jewel_servo;

    public double glyphLeftPos;
    public double glyphRightPos;
    public double jewelPos;


    @Override
    public void init() {

        glyphServoRight = hardwareMap.servo.get("glyphServoRight");
        glyphServoLeft = hardwareMap.servo.get("glyphServoLeft");
        jewel_servo = hardwareMap.servo.get("jewelServo");

        glyphServoLeft.setPosition(0.5);
        glyphServoRight.setPosition(1.0);
        jewel_servo.setPosition(0.5);
    }

    @Override
    public void loop() {

      //  moveServos();
      incrementClose();
      incrementOpen();
        glyphLeftPos = glyphServoLeft.getPosition();
        glyphRightPos = glyphServoRight.getPosition();
        jewelPos = jewel_servo.getPosition();

        telemetry.addData("right stick is", gamepad1.right_stick_y);
        telemetry.addData("left stick is", gamepad1.left_stick_y);
       // telemetry.addData("jewel pos", jewel_servo.getPosition());
        telemetry.update();

    }

    public void sleep(long time){
        long startTime = System.currentTimeMillis();
        long  endTime = startTime + time;

        while(startTime < endTime){

        }
    }


    public void incrementOpen(){

        while (gamepad1.x){
            glyphServoLeft.setPosition(glyphServoLeft.getPosition()+0.05);
            glyphServoRight.setPosition(glyphServoRight.getPosition()-0.05);
            sleep(500);
        }
    }
    public void incrementClose(){

        while (gamepad1.y) {
            glyphServoLeft.setPosition(glyphServoLeft.getPosition()-0.05);
            glyphServoRight.setPosition(glyphServoRight.getPosition()+0.05);
            sleep(500);
        }
    }
}