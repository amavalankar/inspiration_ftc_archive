package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Depot")
public class Depot extends ExtendedLinearOpMode {

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
        telemetry.addLine("Initialization tdone ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        // Dehang
        moveActuator(5, 6);

        // Unhook
        encoderStrafe(-13, 0.25);
        doEncoderTurn(0.25, 10);

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();

        telemetry.addData("Current position is", robot.getSamplingOrder());
        telemetry.update();

        switch (goldLocation) {

            case LEFT:

                encoderDriveIN(-12, -12, 1, 3);
                leftSample();
                encoderDriveIN(4, 4, 0.5, 3);

                encoderStrafe(-50, 1);
                doEncoderTurn(0.5, 45);

                encoderDriveIN(-50, -50, 0.5, 5);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                encoderDriveIN(90, 90, 1, 5);

                break;

            case CENTER:

                encoderDriveIN(-12, -12, 1, 3);
                centerSample();
                encoderDriveIN(4, 4, 0.5, 3);

                encoderStrafe(-70, 1);
                doEncoderTurn(0.5, 45);

                encoderDriveIN(-50, -50, 0.5, 5);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                encoderDriveIN(90, 90, 1, 5);

                break;

            case RIGHT:

                encoderDriveIN(-12, -12, 1, 3);
                rightSample();
                encoderDriveIN(4, 4, 0.5, 3);

                encoderStrafe(-90, 1);
                doEncoderTurn(0.5, 45);

                encoderDriveIN(-50, -50, 0.5, 5);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);

                encoderDriveIN(90, 90, 1, 5);

                break;

        }


        robot.disableVision();

        /**
         * END OF AUTONOMOUS COMMON AHHHH
         */
    }

}
