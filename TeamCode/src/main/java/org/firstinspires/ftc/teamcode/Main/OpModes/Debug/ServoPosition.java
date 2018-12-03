package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Hardware.Subcomponents.Servo;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedOpMode;

/**
 * Created by adityamavalankar on 12/1/18.
 */

@TeleOp(name = "Find Servo Position", group = "Debug")
public class ServoPosition extends ExtendedOpMode {

    com.qualcomm.robotcore.hardware.Servo testServo;
    String hwmapName = "servo";

    @Override
    public void init() {
        testServo = hardwareMap.servo.get(hwmapName);
        telemetry.addLine("Ready to go!");
        testServo.setPosition(0.5);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            testServo.setPosition(testServo.getPosition()-0.1);
            sleep(400);
        }

        else if (gamepad1.right_bumper) {
            testServo.setPosition(testServo.getPosition()+0.1);
            sleep(400);
        }

        telemetry.addData("Current Position of Servo", testServo.getPosition());
    }

}
