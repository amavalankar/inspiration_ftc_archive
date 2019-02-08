package org.firstinspires.ftc.teamcode.OpModes.Autonomous.ModifiedAuto;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.CameraCropAngle;

/**
 * January 26, 2019
 * FINAL PROGRAM FOR LEAGUE MEET THREE
 * STATUS: WORKING CONSISTENTLY :)
 *
 *
 * CRATER SIDE:
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

@Disabled
@Autonomous(name = "Qualificaion Match #7")
public class QualificationMatchSeven extends ExtendedLinearOpMode {

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
        //Disable vision earlier because it should

        // --Sample + Drive to Wall-- \\

        switch (goldLocation) {

            case LEFT:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                leftSample();

                // Turn from sampling position and drive into wall
                doEncoderTurn(0.5, -90);
                encoderDriveIN(-20, -20, 1, 5);

                // --Drop Marker + Crater-- \\

                // Turn towards depot and strafe towards the wall
                doEncoderTurn(1, -45);
                encoderStrafeOffset(40, 1, 0, 0.6,4);

                // Drive towards depot and drop marker while rolling against the wall
                encoderDriveIN(-33, -33, 1, 10);
                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                // Correct angle by turning slightly and drive into crater: drivetrain commits seppuku
                doEncoderTurn(0.7, -5);
                encoderDriveIN(70, 70, 0.7, 5);

                break;

            case CENTER:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                centerSample();

                // Drive into wall and turn
                doEncoderTurn(0.5, -90);
                encoderDriveIN(-40, -40, 1, 5);

                // --Drop Marker + Crater-- \\

                // Turn towards depot and strafe towards the wall
                doEncoderTurn(1, -45);
                encoderStrafeOffset(40, 1, 0, 0.6,4);

                // Drive towards depot and drop marker while rolling against the wall
                encoderDriveIN(-33, -33, 1, 10);
                tiltMarker(1.2, -0.8);
                sleep(450);
                tiltMarker(2, 0.8);

                // Correct angle by turning slightly and drive into crater: drivetrain commits seppuku
                doEncoderTurn(0.7, -5);
                encoderDriveIN(70, 70, 0.7, 5);

                break;

            case RIGHT:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                rightSample();

                // Drive into wall and turn
                encoderDriveIN(-27, -27, 1, 5);


                break;

            case UNKNOWN:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                rightSample();

                // Drive into wall and turn
                encoderDriveIN(-27, -27, 1, 5);


                break;
        }






    }

}
