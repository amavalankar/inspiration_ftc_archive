package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Avocado Common Auto")
public class AutonomousCommon extends ExtendedLinearOpMode {

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
        // Telemetry confirms successful initialization. It's delayed to let everything load
        sleep(3000);
        telemetry.addLine("Initialization tdone ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        // Dehang
        moveActuator(6, 6);
        sleep(300);

        // Unhook
        resetEncoderAngle();
        doEncoderTurn(0.25, 90);
        encoderDriveIN(4, 4, 0.25, 5);


        // Turn to sample
        doEncoderTurn(-0.25, 90);

        //save sampling order of minerals to this variable
        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        sleep(400);

        while (opModeIsActive()) {
            telemetry.addData("Current Orientation is", robot.getSamplingOrder());
            telemetry.update();
        }

//        switch (robot.getSamplingOrder()) {
//
//            case LEFT:
//
//                leftSample();
//
//            case CENTER:
//
//                centerSample();
//
//            case RIGHT:
//
//                rightSample();
//
//            case UNKNOWN:
//
//                telemetry.addLine("Hah too bad for you, the robot can't find ANYTHING.");
//                telemetry.update();
//                rightSample();
//
//        }

        robot.disableVision();

        /**
         * END OF AUTONOMOUS COMMON AHHHH
         */
    }

}
