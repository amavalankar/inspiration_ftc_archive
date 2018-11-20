package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.Hardware.Subcomponents.Motor;
import org.firstinspires.ftc.teamcode.Salsa.Robots.Asteroid;

/**
 * Created by adityamavalankar on 11/15/18.
 */

@TeleOp(name = "test Actuator")
public class TestActuator extends OpMode {

    Asteroid robot = new Asteroid();
    Motor actuator;

    @Override
    public void init() {

        robot.robot.initDrivetrain();
        actuator.init("actuator", hardwareMap);

    }

    @Override
    public void loop() {

        robot.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        actuatorMove();

    }

    public void actuatorMove() {
        if(gamepad1.dpad_up) {
            actuator.setPower(1);
        }

        else if(gamepad1.dpad_down) {
            actuator.setPower(-1);
        }
        else {
            actuator.setPower(0);
        }
    }
}
