package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

@Autonomous(name = "Just Hang")
public class JustHang extends SalsaLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
        setHardwareMap(hardwareMap);
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.liftSlides = hardwareMap.dcMotor.get(constants.LIFT_SLIDES_NAME);
        robot.initVision();
        robot.enableVision();
        // Telemetry confirms successful initialization
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        // Align
//        alignTape();

//        robot.liftSlides.setPower(-1);
//
//        while(robot.leftLineDist.getDistance(DistanceUnit.INCH) > 2 || robot.rightLineDist.getDistance(DistanceUnit.INCH) > 2) {
//
//            robot.liftSlides.setPower(-1);
//
//        }

        moveActuator(-5.25);
        sleep(1500);

        robot.liftSlides.setPower(0);
        sleep(100);

        encoderDriveIN(-4, -4, 0.7, 3);
        sleep(100);

        ElapsedTime loopTime = new ElapsedTime();
        loopTime.reset();

        while (loopTime.seconds() + 2.5 > System.currentTimeMillis()) {
            robot.liftSlides.setPower(11);
        }

        sleep(100);
        encoderDriveIN(7, 7, 0.6, 4);


        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        sleep(400);

        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {

            encoderDriveIN(1, 1, 0.6, 2);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER || goldLocation == SamplingOrderDetector.GoldLocation.UNKNOWN) {

            encoderDriveIN(-5, -5, 0.6, 4);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {

            encoderDriveIN(-9, -9, 0.6, 6);

        }

        sleep(250);

        encoderTurn(0.25, 90);

        sleep(250);

        encoderDriveIN(20, 20, 0.6, 4);

        sleep(250);

        encoderDriveIN(-9, -9, 0.6, 4);

        sleep(250);

        encoderTurn(0.25, -90);

        sleep(250);

        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {

            encoderDriveIN(-6, -6, 0.6, 2);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {

            encoderDriveIN(4, 4, 0.6, 6);

        }


    }

}
