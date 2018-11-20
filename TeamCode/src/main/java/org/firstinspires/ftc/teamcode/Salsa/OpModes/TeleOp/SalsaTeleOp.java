package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaOpMode;

/**
 * Created by adityamavalankar on 11/19/18.
 */

@TeleOp(name = "Salsa TeleOp BETA", group = "Salsa")
public class SalsaTeleOp extends SalsaOpMode {

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();

    }

    @Override
    public void loop() {
        drive(gamepad1.left_stick_y, gamepad2.right_stick_y);

    }
}
