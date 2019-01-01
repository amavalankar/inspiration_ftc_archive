package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Avocado Crater Drive DO HANG BASIC")
public class AutonomousDriveCraterBasic extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
        setHardwareMap(hardwareMap);
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.liftSlides = hardwareMap.dcMotor.get(constants.LIFT_SLIDES_NAME);
        robot.initVision(CameraCropAngle.LEFT);
        robot.enableVision();
        robot.markerDepositer = hardwareMap.servo.get("markerDepositer");
        // Telemetry confirms successful initialization. It's delayed to let everything load
        sleep(3000);

        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();


        waitForStart();
        resetEncoderAngle();

        // Dehang
        moveActuator(-6, 6.7);
        sleep(750);
        // Unhook
        resetEncoderAngle();
        encoderDriveINNew(5, 5, 0.25, 3);
        sleep(100);
        moveActuator(2, 2.5);
        encoderDriveINNew(-8,-8,0.25,3);

        //save sampling order of minerals to this variable
        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        sleep(400);

        telemetry.addData("Current Orientation is", robot.getSamplingOrder());
        telemetry.update();

        switch (robot.getSamplingOrder()) {

            case LEFT:

                telemetry.addLine("LEFT GOLD.");
                telemetry.update();
                encoderDriveIN(-1, -1, 0.5, 5);
                encoderTurn(0.5, -110);
                encoderDriveIN(20, 20, 0.5, 5);
                break;

            case CENTER:

                telemetry.addLine("CENTER GOLD.");
                telemetry.update();
                encoderDriveIN(4, 4, 0.5, 5);
                encoderTurn(0.5, -90);
                encoderDriveIN(20, 20, 0.5, 5);
                break;

            case RIGHT:

                telemetry.addLine("RIGHT GOLD");
                telemetry.update();
                encoderDriveIN(14, 14, 0.5, 5);
                encoderTurn(0.5, -90);
                encoderDriveIN(20, 20, 0.5, 5);
                break;

            case UNKNOWN:

                telemetry.addLine("Hah too bad for you, the robot can't find ANYTHING.");
                telemetry.update();
                encoderDriveIN(4, 4, 0.5, 5);
                encoderTurn(0.5, -90);
                encoderDriveIN(20, 20, 0.5, 5);
                break;

        }

        robot.disableVision();

        encoderDriveINNew(10, 10, 0.25, 4);

        /**
         * END OF AUTONOMOUS COMMON AHHHH
         */

        robot.markerDepositer.setPosition(0);

    }
}
