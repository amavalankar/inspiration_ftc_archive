package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.CameraCropAngle;

@Autonomous(name = "Depot")
public class Depot extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
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

                encoderDriveIN(-12, -12, 1, 3);
                leftSample();

                encoderStrafeOffset(-50, 1, 0, 0.6);
                doEncoderTurn(0.5, 45);

                encoderDriveIN(-26, -26, 0.5, 5);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                break;

            case CENTER:

                encoderDriveIN(-12, -12, 1, 3);
                centerSample();

                encoderStrafeOffset(-70, 1, 0, 0.6);
                doEncoderTurn(0.5, 45);

                encoderDriveIN(-26, -26, 0.5, 5);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                break;

            case RIGHT:

                encoderDriveIN(-12, -12, 1, 3);
                rightSample();

                encoderStrafeOffset(-90, 1, 0, 0.6);
                doEncoderTurn(0.5, 45);

                encoderDriveIN(-26, -26, 0.5, 5);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                break;

        }

        robot.disableVision();

        /**
         * END OF AUTONOMOUS COMMON AHHHH
         */
    }

}
