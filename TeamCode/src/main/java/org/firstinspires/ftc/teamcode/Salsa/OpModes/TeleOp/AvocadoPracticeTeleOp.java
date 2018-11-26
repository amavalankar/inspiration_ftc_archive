package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Salsa.Hardware.Subcomponents.Motor;
import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaOpMode;

@TeleOp(name = "Avocado TeleOp Practice", group = "Avocado")
public class AvocadoPracticeTeleOp extends SalsaOpMode {


    @Override
    public void init() {

        // Initiate all the hardwareMap names
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initTiltingMechanism();
        robot.initHanger();


    }

    @Override
    public void loop() {

        // Run TeleOP methods
        drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        hang(gamepad2.dpad_up, gamepad2.dpad_down);
        strafe(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, 0.5);

    }
}
