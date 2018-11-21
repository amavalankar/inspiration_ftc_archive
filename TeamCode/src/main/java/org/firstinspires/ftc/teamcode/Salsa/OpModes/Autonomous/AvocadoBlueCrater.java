package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

/**
 * Created by adityamavalankar on 11/20/18.
 */

public class AvocadoBlueCrater extends SalsaLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrainAvocado();
        robot.initTiltingMechanism();

        robot.enableVision();
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();

        sleep(50);

        doSampling();

        robot.disableVision();

    }

    //hello
}
