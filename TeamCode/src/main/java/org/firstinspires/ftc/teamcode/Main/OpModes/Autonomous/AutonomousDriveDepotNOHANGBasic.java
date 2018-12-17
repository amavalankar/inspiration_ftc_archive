package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Avocado Depot Drive DO HANG BASIC")
public class AutonomousDriveDepotNOHANGBasic extends ExtendedLinearOpMode {

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
        SamplingOrderDetector.GoldLocation goldLocation;
        // Telemetry confirms successful initialization. It's delayed to let everything load
        sleep(3000);
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();


        waitForStart();
        resetEncoderAngle();

        goldLocation = robot.getSamplingOrder();
        sleep(400);

//        // Dehang
//        moveActuator(-6);
//        sleep(500);
//        // Unhook
//        resetEncoderAngle();
//        encoderDriveINNew(-5, -5, 0.25, 3);
//        sleep(100);
//        moveActuator(2);

        // Dehang
        moveActuator(-6, 6.15);
        sleep(750);
        // Unhook
        resetEncoderAngle();
        encoderDriveINNew(-5, -5, 0.25, 3);
        sleep(100);
        moveActuator(2, 2.5);

        //save sampling order of minerals to this variable
        goldLocation = robot.getSamplingOrder();
        sleep(400);

        telemetry.addData("Current Orientation is", goldLocation);
        telemetry.update();

        switch (goldLocation) {

            case LEFT:

                telemetry.addLine("LEFT GOLD.");
                telemetry.update();
                encoderTurn(0.5, -110);
                encoderDriveIN(20, 20, 0.5, 5);
                encoderTurn(0.3, -120);
                encoderDriveINNew(10, 10, 0.25, 4);
                robot.markerDepositer.setPosition(0);
                sleep(1000);
                encoderDriveINNew(-10, -10, 0.25, 4);
                encoderTurn(0.5, -90);
                encoderDriveINNew(-35, -35, 0.3, 6);
//                encoderTurn(0.5, 5);
//                encoderDriveINNew(30, 30, 0.4, 8);
                break;

            case CENTER:

                telemetry.addLine("CENTER GOLD.");
                telemetry.update();
                encoderDriveIN(4, 4, 0.5, 5);
                encoderTurn(0.5, -90);
                encoderDriveIN(30, 30, 0.5, 5);
                robot.markerDepositer.setPosition(0);
                sleep(500);
                encoderDriveINNew(-35, -35, 0.3, 6);
//                encoderTurn(0.5, 5);
//                encoderDriveINNew(45, 45, 0.4, 8);
                break;

            case RIGHT:

                telemetry.addLine("RIGHT GOLD");
                telemetry.update();
                encoderDriveIN(14, 14, 0.5, 5);
                encoderTurn(0.5, -90);
                encoderDriveIN(20, 20, 0.5, 5);
                encoderTurn(0.3, -45);
                encoderDriveINNew(10, 10, 0.25, 4);
                robot.markerDepositer.setPosition(0);
                sleep(1000);
                encoderDriveINNew(-10, -10, 0.25, 4);
                encoderTurn(0.5, -90);
                encoderDriveINNew(-35, -35, 0.3, 6);
//                encoderTurn(0.5, 0);
//                encoderDriveINNew(50, 50, 0.4, 8);

                break;

            case UNKNOWN:

                telemetry.addLine("CENTER GOLD.");
                telemetry.update();
                encoderDriveIN(4, 4, 0.5, 5);
                encoderTurn(0.5, -90);
                encoderDriveIN(30, 30, 0.5, 5);
                robot.markerDepositer.setPosition(0);
                sleep(500);
                encoderDriveINNew(-35, -35, 0.3, 6);
//                encoderTurn(0.5, 5);
//                encoderDriveINNew(45, 45, 0.4, 8);
                break;
        }

        robot.disableVision();
//
//
//        switch (goldLocation) {
//            case LEFT:
//                encoderTurn(0.25, -100);
//
//                setPower(0.25);
//                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
//                }
//                setPower(0);
//
//                encoderTurn(-0.25, -150);
//
//                robot.markerDepositer.setPosition(0);
//                sleep(1500);
//
//                encoderDriveINNew(-10, -10, 0.4, 5);
//
//                encoderTurn(0.25, 25);
//
//                encoderDriveINNew(40, 40, 0.4, 10);
//                break;
//
//            case CENTER:
//                encoderDriveINNew(5, 5, 0.4, 5);
//
//                robot.markerDepositer.setPosition(0);
//                sleep(1500);
//
//                encoderDriveINNew(-25, -25, 0.3, 5);
//
////                encoderTurn(0.25, 0);
////
////                setPower(0.25);
////                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
////                }
////                setPower(0);
////
////                encoderTurn(0.25, 30);
////
////                encoderDriveINNew(8, 8, 0.3, 5);
//
//                break;
//
//            case RIGHT:
//                encoderTurn(0.25, -40);
//
//                setPower(0.25);
//                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
//                }
//                setPower(0);
//
//                encoderTurn(-0.25, -150);
//
//                robot.markerDepositer.setPosition(0);
//                sleep(1500);
//
//                encoderTurn(0.25, -40);
//
//                encoderDriveINNew(-8, -8, 0.4, 5);
//                break;
//
//        }

    }
}
