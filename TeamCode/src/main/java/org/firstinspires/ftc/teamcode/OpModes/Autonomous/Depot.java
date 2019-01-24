package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.CameraCropAngle;

/**
 *
 * DEPOT SIDE:
 * A wise man named Colin once said, "Scratch once, scratch forever..."
 *
 * This autonomous program not only attempts to scratch the side walls,
 * but also aims to gain as many points as possible by
 * DEHANGING, SAMPLING, DROPPING OFF THE TEAM MARKER, AND PARTIALLY PARKING.
 *
 * This file inherits ExtendedLinearOpMode, from which it obtains all the movement functions.
 * ExtendedLinearOpMode is a child class of LinearOpMode. ExtendedLinearOpMode creates objects
 * of classes such as Robot and Constants.
 *
 */


@Autonomous(name = "Depot")
public class Depot extends ExtendedLinearOpMode {

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
        robot.initColorSensors();
        robot.initVision(CameraCropAngle.LEFT);
        robot.enableVision();

        // Telemetry confirms successful initialization.
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested() && !robot.startPressed) {
            telemetry.addLine("(Fix) Initialization done ... Ready to start!");
            telemetry.update();

            if(isStopRequested()) {
                robot.startPressed = true;
            }
        }

//        if (robot.PHONE_MANUFACTURER.equals("motorola")) {
//
//            // Due to an error, seemingly unique to motorola phones, we need to constantly send updates between the phone, so we
//            // loop a telemetry message being sent to the RC
//            while (!opModeIsActive() && !isStopRequested() && !robot.startPressed) {
//                telemetry.addLine("(Motorola) Initialization done ... Ready to start!");
//                telemetry.update();
//
//                if(isStopRequested()) {
//                    robot.startPressed = true;
//                }
//            }
//
//        }
//
//        // This is meant for other phones, but if the error persists on other phones , the while loop used above may be universal
//        else {
//            waitForStart();
//        }


        // --Dehang + Unhook-- \\

        resetEncoderAngle();
        moveActuator(5, 4);
        encoderStrafeOffset(-20, 1, 0, 0.6);
        doEncoderTurn(0.25, 10);


        // --Vision-- \\

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        telemetry.addData("Current position is", robot.getSamplingOrder());
        telemetry.update();


        // --Sample + Drive to Wall-- \\

        switch (goldLocation) {

            case LEFT:

                encoderDriveIN(-12, -12, 1, 3);
                leftSample();
                encoderStrafeOffset(-50, 1, 0, 0.6);

                break;

            case CENTER:

                encoderDriveIN(-12, -12, 1, 3);
                centerSample();
                encoderStrafeOffset(-70, 1, 0, 0.6);


                break;

            case RIGHT:

                encoderDriveIN(-12, -12, 1, 3);
                rightSample();
                encoderStrafeOffset(-90, 1, 0, 0.6);

                break;

            case UNKNOWN:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                rightSample();

                // Drive into wall and turn
                doEncoderTurn(0.5, 90);
                encoderDriveIN(60, 60, 1, 5);

                break;
        }

        doEncoderTurn(0.5, 45);
        encoderDriveIN(-26, -26, 0.5, 5);
        tiltMarker(1, -0.5);
        tiltMarker(1, 0.5);

        robot.disableVision();

    }

}
