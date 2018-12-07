package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;

/**
 * Created by adityamavalankar on 12/2/18.
 */
@Disabled
@Autonomous(name = "Test Robot Turn", group = "Debug")
public class RobotTurn extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initGyro();

        sleep(500);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();
        resetGyro();

        sleep(1500);

        gyroTurn(0.2, 90, 4);


    }
}
