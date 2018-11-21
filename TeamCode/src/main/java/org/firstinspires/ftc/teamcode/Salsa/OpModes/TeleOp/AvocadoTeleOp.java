package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.Hardware.Subcomponents.Motor;
import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaOpMode;

/**
 * Created by adityamavalankar on 11/19/18.
 */

@TeleOp(name = "Avocado TeleOp Main TEST", group = "Avocado")
public class AvocadoTeleOp extends SalsaOpMode {


    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrainAvocado();
        robot.initTiltingMechanism();
    }

    @Override
    public void loop() {
        drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        robot.extension.setPower(gamepad2.left_stick_y);
        robot.tiltMotor.setPower(gamepad2.right_stick_y);

    }
}
