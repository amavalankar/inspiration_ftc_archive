package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaOpMode;

/**
 * Created by adityamavalankar on 11/19/18.
 */

@TeleOp(name="test OpMode Extension")
public class TestOpModeExtension extends SalsaOpMode {

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initVision();
    }

    @Override
    public void loop() {
    }
}
