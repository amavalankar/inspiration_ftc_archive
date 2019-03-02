package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Debug;

import android.os.Build;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.CameraCropAngle;

/**
 * January 26, 2019
 * FINAL PROGRAM FOR LEAGUE MEET THREE
 * STATUS: WORKING CONSISTENTLY :)
 *
 *
 * CRATER SIDE:
 * DEHANGING, SAMPLING, DROPPING OFF THE TEAM MARKER, AND PARTIALLY PARKING.
 *
 * This file inherits ExtendedLinearOpMode, from which it obtains all the movement functions.
 * ExtendedLinearOpMode is a child class of LinearOpMode. ExtendedLinearOpMode creates objects
 * of classes such as Robot and Constants.
 *
 */


@Autonomous(name = "Vision")
public class Vision extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        /* --Initialization-- */

        // Mechanisms
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initServo();
        robot.initHanger();
        robot.initTiltingMechanism();

        // Sensors
        robot.initVision(CameraCropAngle.LEFT);
        robot.enableVision();


        while (!opModeIsActive() && !isStopRequested() && !robot.startPressed) {
            telemetry.addLine("(Fix) Initialization done ... Ready to start!");

            if (isStopRequested()) {
                robot.startPressed = true;
            }

            telemetry.addLine(robot.returnPhoneOrientation());
            telemetry.update();
        }

        // --Vision-- \\

        while (opModeIsActive()) {
            SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
            telemetry.addData("Current position is", robot.getSamplingOrder());
            telemetry.update();
        }
    }

}