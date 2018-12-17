package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Disabled
@Autonomous(name = "Avocado Depot Drive")
public class AutonomousDriveDepot extends ExtendedLinearOpMode {

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

        SamplingOrderDetector.GoldLocation goldLocation = SamplingOrderDetector.GoldLocation.CENTER;


        waitForStart();
        resetEncoderAngle();

        goldLocation = robot.getSamplingOrder();
        sleep(400);

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
                setPower(0.25);
                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
                }
                setPower(0);

                sleep(100);

                encoderTurn(0.25, -45);

                setPower(0.25);
                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
                }
                setPower(0);

                robot.markerDepositer.setPosition(0);
                sleep(1500);

                encoderDriveINNew(-10, -10, 0.4, 5);

                encoderTurn(0.3, -173);

                encoderDriveINNew(60, 60, 0.4, 10);
                break;


            case CENTER:
                setPower(0.25);
                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
                }
                setPower(0);

                sleep(100);


                robot.markerDepositer.setPosition(0);
                sleep(1500);

                encoderDriveINNew(-10, -10, 0.4, 5);

                encoderTurn(0.3, -170);

                encoderDriveINNew(60, 60, 0.4, 10);
                break;

            case RIGHT:
                setPower(0.25);
                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
                }
                setPower(0);

                sleep(100);

                encoderTurn(0.25, -135);

                setPower(0.25);
                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
                }
                setPower(0);
                break;

            case UNKNOWN:
                setPower(0.25);
                while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
                }
                setPower(0);

                sleep(100);
                break;




        }


    }
}
