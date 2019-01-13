package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Crater")
public class Crater extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
        setHardwareMap(hardwareMap);
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.liftSlides = hardwareMap.dcMotor.get(constants.LIFT_SLIDES_NAME);
        robot.initVision(CameraCropAngle.LEFT);
        robot.initServo();
        robot.enableVision();
        robot.initTiltingMechanism();
        // Telemetry confirms successful initialization. It's delayed to let everything load
        sleep(3000);
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        // Dehang
        moveActuator(5, 4);

        // Unhook
        encoderStrafeOffset(-20, 1, 0, 0.6);
        doEncoderTurn(0.25, 10);

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();

        telemetry.addData("Current position is", robot.getSamplingOrder());
        telemetry.update();

        switch (goldLocation) {

            case LEFT:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                leftSample();

                // Drive into wall and turn
                doEncoderTurn(0.5, 90);
                encoderDriveIN(20, 20, 1, 5);
                doEncoderTurn(1, 135);

                // Strafe closer to wall
                encoderStrafeOffset(40, 1, 0, 0.6);

                // Drive towards depot and drop marker
                encoderDriveIN(-30, -30, 1, 10);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                // Drive into crater: drivetrain commits seppuku
                encoderTurn(1, -10);
                encoderDriveIN(70, 70, 0.5, 10);

                break;

            case CENTER:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                centerSample();

                // Drive into wall and turn
                doEncoderTurn(0.5, 90);
                encoderDriveIN(40, 40, 1, 5);
                doEncoderTurn(1, 135);

                // Strafe closer to wall
                encoderStrafeOffset(40, 1, 0, 0.6);

                // Drive towards depot and drop marker
                encoderDriveIN(-30, -30, 1, 10);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                // Drive into crater: drivetrain commits seppuku
                encoderTurn(1, -10);
                encoderDriveIN(70, 70, 0.5, 10);

                break;

            case RIGHT:

                // Push the mineral off
                encoderDriveIN(-12, -12, 1, 3);
                rightSample();

                // Drive into wall and turn
                doEncoderTurn(0.5, 90);
                encoderDriveIN(60, 60, 1, 5);
                doEncoderTurn(1, 135);

                // Strafe closer to wall
                encoderStrafeOffset(40, 1, 0, 0.6);

                // Drive towards depot and drop marker
                encoderDriveIN(-30, -30, 1, 10);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                // Correct angle and drive into crater: drivetrain commits seppuku
                encoderTurn(1, -10);
                encoderDriveIN(70, 70, 0.5, 10);

                break;

        }


        robot.disableVision();

        /**
         * END OF AUTONOMOUS COMMON AHHHH
         */
    }

}
