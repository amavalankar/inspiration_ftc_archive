package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.Robots.Asteroid;

/**
 * Created by adityamavalankar on 11/4/18.
 */

@TeleOp(name = "Four Wheel Drive salsa", group = "Salsa")
public class AsteroidDrive extends OpMode {

    public Asteroid asteroid = new Asteroid();

    @Override
    public void init() {
        asteroid.robot.initDrivetrain();
    }

    @Override
    public void loop() {
        asteroid.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
    }
}
