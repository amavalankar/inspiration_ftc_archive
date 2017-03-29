package org.firstinspires.ftc.teamcode.Steven;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Steven on 3/21/2017.
 */
@TeleOp(name = "linearservotest", group = "")
public class LinearServoTest extends OpMode {

    ElapsedTime runtime = new ElapsedTime();
    Servo linearservo;
    public void init() {
        linearservo = hardwareMap.servo.get("linearServo");
        linearservo.setPosition(0);
        telemetry.addData("","READY TO START");
    }

    @Override
    public void start(){
       // runtime.reset();
        linearservo.setPosition(0);
      //  telemetry.addData("Time",runtime.milliseconds());
       // telemetry.update();
       // runtime.reset();

    }

    @Override
    public void loop(){

    }
}