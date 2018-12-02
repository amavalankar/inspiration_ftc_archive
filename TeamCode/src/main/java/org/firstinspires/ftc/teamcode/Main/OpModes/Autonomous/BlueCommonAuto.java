package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;

@Autonomous(name = "Blue Common Auto")
public class BlueCommonAuto extends ExtendedLinearOpMode {

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

        moveActuator(-6);
        sleep(1500);
        //come down, but still not unhooked

        encoderDriveIN(-4, -4, 0.3, 3);
        sleep(100);
        //drive backwards to unhook

        ElapsedTime loopTime = new ElapsedTime();
        loopTime.reset();

        while (loopTime.seconds() + 3.5 > System.currentTimeMillis()) {
            robot.liftSlides.setPower(1);
        }
        //move actuator down for 3 sec

        sleep(100);
        encoderDriveIN(7, 7, 0.3, 4);
        //drive forward to be in view of the right two minerals

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        sleep(400);

        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {

            encoderDriveIN(1, 1, 0.35, 2);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER || goldLocation == SamplingOrderDetector.GoldLocation.UNKNOWN) {

            encoderDriveIN(-5, -5, 0.35, 4);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {

            encoderDriveIN(-9, -9, 0.35, 6);

        }
        //get gold position, and then drive forwards, or reverse, accordingly to be horozontally aligned with the gold

        sleep(250);

        encoderTurn(0.25, 90);
        //turn to face gold

        sleep(250);

        encoderDriveIN(20, 20, 0.35, 4);
        //drive forward to push gold off

        sleep(250);

        //drive backwards to be behind mineral
        encoderDriveIN(-9, -9, 0.35, 4);

        sleep(250);

        //turn to face wall
        encoderTurn(0.175, -90);
        //align so that we are at the center mineral
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {
            encoderDriveIN(-6, -6, 0.35, 2);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {
            encoderDriveIN(4, 4, 0.35, 6);

        }


    }

}
