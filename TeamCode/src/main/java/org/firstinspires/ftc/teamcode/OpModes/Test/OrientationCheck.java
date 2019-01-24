package org.firstinspires.ftc.teamcode.OpModes.Test;

import org.firstinspires.ftc.teamcode.OpModes.ExtendedLinearOpMode;

/**
 * Created by adityamavalankar on 1/23/19.
 */

public class OrientationCheck extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        robot.setHardwareMap(hardwareMap);


        while (!opModeIsActive() && !isStopRequested() && !robot.startPressed) {
            telemetry.addLine("(Fix) Initialization done ... Ready to start!");

            if(isStopRequested()) {
                robot.startPressed = true;
            }

            /**
             * The following if statement checks whether or not the RC is portrait
             * Please refer to the isLandscape() boolean in the robot.java class
             */
            if (robot.isLandscape()) {
                telemetry.addLine("RC is Landscape");
            } else {
                telemetry.addLine("WARNING: RC is Portrait");
            }
            telemetry.update();
        }

    }
}
