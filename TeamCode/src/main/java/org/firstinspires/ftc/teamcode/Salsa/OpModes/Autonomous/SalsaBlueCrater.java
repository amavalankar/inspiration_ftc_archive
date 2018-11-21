package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Salsa.Constants;
import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;
import org.firstinspires.ftc.teamcode.Salsa.Vision.SamplingDetector;

/**
 * Created by adityamavalankar on 11/19/18.
 */

@Autonomous(name = "Salsa Blue Crater BETA", group = "Salsa")
public class SalsaBlueCrater extends SalsaLinearOpMode {

    /**
     * This is Salsa Blue Crater! As of now, we are currently sampling, and then turning and going
     * forward to knock off the correct gold cube
     */
    SamplingDetector detector = new SamplingDetector();

    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();

        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initVision();

        robot.enableVision();
        robot.initGyro();
        robot.resetGyro();

        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();

        sleep(50);

        doSampling();
        robot.disableVision();

        encoderDriveIN(-10, -10, 0.8, 2);
        sleep(100);
        gyroTurn(0.6, 90, 4);


    }


}
