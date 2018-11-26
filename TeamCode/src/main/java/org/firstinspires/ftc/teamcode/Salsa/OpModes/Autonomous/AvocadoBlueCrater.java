package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

/**
 * Created by adityamavalankar on 11/20/18.
 */

@Autonomous(name = "Avocado Blue Crater BETA")
public class AvocadoBlueCrater extends SalsaLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // INIT
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initTiltingMechanism();
        robot.initVision();
        robot.enableVision();
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();
        waitForStart();
        resetEncoderAngle();

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        telemetry.addLine("Sample");
        telemetry.update();
        sleep(500);
        sleep(50);
        encoderTurn(0.25, 90);


        sleep(50);
        resetEncoderAngle();

        doSampling(goldLocation);

        robot.disableVision();


        encoderTurn(0.25, 0);
        sleep(100);
        encoderDriveIN(-6, -6, 0.6, 3);
        sleep(100);
        encoderTurn(0.25, -90);
        sleep(100);


        encoderDriveIN(15, 15, 0.6, 3);
    }
}
