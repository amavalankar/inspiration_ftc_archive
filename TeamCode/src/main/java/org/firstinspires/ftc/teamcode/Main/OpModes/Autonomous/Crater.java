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
        encoderStrafeOffset(-13, 0.25, 0, 0.5);
        doEncoderTurn(0.25, 10);

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();

        telemetry.addData("Current position is", robot.getSamplingOrder());
        telemetry.update();


        switch (goldLocation) {

            case LEFT:

                encoderDriveIN(-12, -12, 1, 3);
                leftSample();

                doEncoderTurn(0.5, 90);
                encoderDriveIN(20, 20, 1, 5);
                doEncoderTurn(1, 135);
                encoderStrafeOffset(10, 1, 0, 0.5);
                encoderDriveIN(-17, -17, 1, 10);

                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);
                encoderDriveIN(90, 90, 1, 10);

                break;

            case CENTER:

                encoderDriveIN(-12, -12, 1, 3);
                centerSample();
                doEncoderTurn(0.5, 90);
                encoderDriveIN(40, 40, 1, 5);
                doEncoderTurn(1, 135);
                encoderStrafeOffset(10, 1, 0, 0.5);
                encoderDriveIN(-17, -17, 1, 10);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);
//                encoderDriveIN(90, 90, 1, 10);

                break;

            case RIGHT:

                encoderDriveIN(-12, -12, 1, 3);
                rightSample();
                doEncoderTurn(0.5, 90);
                encoderDriveIN(60, 60, 1, 5);
                doEncoderTurn(1, 135);
                encoderStrafeOffset(10, 1, 0, 0.5);

                encoderDriveIN(-17, -17, 1, 10);
                tiltMarker(1, -0.5);
                tiltMarker(1, 0.5);
//                encoderDriveIN(90, 90, 1, 10);

                break;

        }

        robot.disableVision();

        /**
         * END OF AUTONOMOUS COMMON AHHHH
         */
    }

}
