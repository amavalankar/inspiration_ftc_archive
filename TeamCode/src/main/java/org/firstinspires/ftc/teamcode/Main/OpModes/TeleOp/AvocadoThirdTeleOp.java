package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedOpMode;

@TeleOp(name = "Avocado Third TeleOp", group = "Avocado")
public class AvocadoThirdTeleOp extends ExtendedOpMode {
//Created by Shruti on 12/26/2018 in order to test whether the dumper servo and locking servos will work

    @Override
    public void init() {

        // Initiate all the hardwareMap names
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initTiltingMechanism();
        robot.initHanger();
        robot.initServo();


    }

    @Override
    public void loop() {

        // Run TeleOP methods
        drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        collect(gamepad1.right_bumper, gamepad1.left_bumper);
        extend(gamepad2.left_stick_y);
        tilt(gamepad2.right_stick_y);
        hang(gamepad2.dpad_up, gamepad2.dpad_down);
        strafe(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, 0.5);
        lockServos(gamepad1.x, gamepad1.y);
        dumperServo(gamepad2.x, gamepad2.y);
    }
}

