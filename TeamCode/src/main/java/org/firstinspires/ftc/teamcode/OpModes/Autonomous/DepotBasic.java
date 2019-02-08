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


@Autonomous(name = "Depot Basic")
public class DepotBasic extends ExtendedLinearOpMode {

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
        encoderStrafeOffset(-20, 1, 0, 0.6,4);
        doEncoderTurn(0.25, 10);


        // --Vision-- \\

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        telemetry.addData("Current position is", robot.getSamplingOrder());
        telemetry.update();

        sleep(100);
        robot.disableVision();

        // --Sample + Drive to Wall-- \\

        switch (goldLocation) {

            case LEFT:

                encoderDriveIN(-12, -12, 1, 3);
                leftSample();
                encoderStrafeOffset(-50, 1, 0, 0.6, 4);

                doEncoderTurn(0.5, 45);
                encoderDriveIN(-31, -31, 0.5, 5);

                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                encoderDriveIN(70,70,0.4,7);

                break;

            case CENTER:

                encoderDriveIN(-12, -12, 1, 3);
                centerSample();
                doEncoderTurn(0.7, -15);
                encoderStrafeOffset(-30, 1, 0, 0.6,4);

                break;

            case RIGHT:

                encoderDriveIN(-12, -12, 1, 3);
                rightSample();
                doEncoderTurn(0.7, -30);
                encoderStrafeOffset(-30, 1, 0, 0.6,4);

                break;

            case UNKNOWN:

                encoderDriveIN(-12, -12, 1, 3);
                leftSample();
                encoderStrafeOffset(-50, 1, 0, 0.6,4);

                doEncoderTurn(0.5, 45);
                encoderDriveIN(-31, -31, 0.5, 5);

                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                encoderDriveIN(70,70,0.4,7);

                break;
        }




    }

}
