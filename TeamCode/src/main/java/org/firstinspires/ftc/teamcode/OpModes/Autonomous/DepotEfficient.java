package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

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
        doEncoderTurn(0.25, 10);


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

                doEncoderTurn(0.6, 35);

                encoderDriveIN(-16, -16, 1, 5);


                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                doEncoderTurn(0.6, 15);

                encoderStrafeOffset(25, 1, 0, 0.6, 2.4);

                break;

            case CENTER:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                centerSampleNoBack();

                encoderDriveIN(-16, -16, 1, 5);


                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                doEncoderTurn(0.6, 48);

                encoderStrafeOffset(-35, 1, 0, 0.6, 2.4);


                break;

            case RIGHT:

                // Push the mineral off
                encoderDriveIN(-13, -13, 1, 3);
                rightSampleNoBack();

                doEncoderTurn(0.6, -20);

                encoderDriveIN(-16, -16, 1, 5);


                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                doEncoderTurn(0.6, 70);

                encoderStrafeOffset(-45, 1, 0, 0.6, 3.5);

                break;

            case UNKNOWN:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                centerSampleNoBack();

                encoderDriveIN(-16, -16, 1, 5);


                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                doEncoderTurn(0.6, 48);

                encoderStrafeOffset(-25, 1, 0, 0.6, 2.4);


                break;
        }

        encoderStrafeOffset(5, 1, 0, 0.6, 1.5);

        // Correct angle by turning slightly and drive into crater: drivetrain commits seppuku
        doEncoderTurn(0.7, 3);
        encoderDriveIN(74, 74, 0.7, 5);



    }

}
