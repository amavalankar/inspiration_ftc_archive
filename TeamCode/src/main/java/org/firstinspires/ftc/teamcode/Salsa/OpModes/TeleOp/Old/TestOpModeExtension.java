package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaOpMode;

/**
 * Created by adityamavalankar on 11/19/18.
 */

@Disabled
@TeleOp(name="test OpMode Extension")
public class TestOpModeExtension extends SalsaOpMode {

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
    }

    @Override
    public void loop() {
    }
}
