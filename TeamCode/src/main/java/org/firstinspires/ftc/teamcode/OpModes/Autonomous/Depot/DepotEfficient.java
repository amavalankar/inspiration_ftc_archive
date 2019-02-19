package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Depot;

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
 * DEPOT SIDE:
 * DEHANGING, SAMPLING, DROPPING OFF THE TEAM MARKER, AND PARTIALLY PARKING.
 *
 * This file inherits ExtendedLinearOpMode, from which it obtains all the movement functions.
 * ExtendedLinearOpMode is a child class of LinearOpMode. ExtendedLinearOpMode creates objects
 * of classes such as Robot and Constants.
 *
 */


@Autonomous(name = "Depot Efficient")
public class DepotEfficient extends ExtendedLinearOpMode {

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

            if(isStopRequested()) {
                robot.startPressed = true;
            }

            telemetry.addLine(robot.returnPhoneOrientation());
            telemetry.update();
        }


        // --Dehang + Unhook-- \\

        resetEncoderAngle();
        moveActuator(5, 4);
        encoderStrafeOffset(-20, 1, 0, 0.6, 4);
        doEncoderTurn(0.25, 6);


        // --Vision-- \\

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        telemetry.addData("Current position is", robot.getSamplingOrder());
        telemetry.update();

        sleep(100);
        robot.disableVision();
        //Disable vision earlier because it should

        // --Sample + Drive to Wall-- \\

        switch (goldLocation) {

            case LEFT:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                leftSampleNoBack();

                doEncoderTurn(0.6, 15);

                encoderDriveIN(-16, -16, 1, 5);

                doEncoderTurn(0.6, 35);

                encoderStrafeOffset(-40, 1, 0, 0.6, 2.9);

                encoderDriveIN(6, 6, 1, 5);

                break;

            case CENTER:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                centerSampleNoBack();

                encoderDriveIN(-16, -16, 1, 5);


                doEncoderTurn(0.6, 48);

                encoderStrafeOffset(-47, 1, 0, 0.6, 3);

                encoderDriveIN(12, 12, 1, 5);


                break;

            case RIGHT:

                // Push the mineral off
                encoderDriveIN(-13, -13, 1, 3);
                rightSampleNoBack();

                doEncoderTurn(0.6, -38);

                encoderDriveIN(-21, -21, 1, 5);

                doEncoderTurn(0.6, 90);

                encoderStrafeOffset(-65, 1, 0, 0.6, 3);

                encoderDriveIN(16, 16, 1, 5);

                break;

            case UNKNOWN:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                leftSampleNoBack();

                doEncoderTurn(0.6, 15);

                encoderDriveIN(-16, -16, 1, 5);

                doEncoderTurn(0.6, 35);

                encoderStrafeOffset(-40, 1, 0, 0.6, 2.9);

                encoderDriveIN(6, 6, 1, 5);

                break;
        }

        encoderStrafeOffset(5, 1, 0, 0.6, 1.5);

        tiltMarker(1.2, -0.8);
        sleep(450);
        tiltMarker(2, 0.8);

        // Correct angle by turning slightly and drive into crater: drivetrain commits seppuku
        doEncoderTurn(0.7, 3);
        encoderDriveIN(87, 87, 0.7, 5);



    }

}
