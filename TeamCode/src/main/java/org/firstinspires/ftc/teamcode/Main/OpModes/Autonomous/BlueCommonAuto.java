package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Blue Common Auto MAIN")
public class BlueCommonAuto extends ExtendedLinearOpMode {

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
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();


        //come down, but still not unhooked
        /*
        moveActuator(4.5);
        sleep(1500);
        */
        //drive backwards to unhook
        resetEncoderAngle();
        encoderDriveINNew(-4, -4, 0.3, 3);
        sleep(100);


        // Bring actuator back down, temporarily commented out due to time constraints.
        // moveActuator(-2);


        //save sampling order of minerals to this variable
        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        sleep(400);

        //get gold position, and then drive forwards, or reverse, accordingly to be horizontally aligned with the gold
        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {

            encoderDriveINNew(1, 1, 0.35, 10);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER || goldLocation == SamplingOrderDetector.GoldLocation.UNKNOWN) {

            encoderDriveINNew(-5, -5, 0.35, 6);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {

            encoderDriveINNew(-9, -9, 0.35, -1);

        }
        sleep(250);

        //turn to face gold
        encoderTurn(0.25, -80);

        //drive forward to push gold off
        encoderDriveINNew(-20, -20, 0.35, 4);




        //drive backwards to be behind mineral
        encoderDriveINNew(9, 9, 0.35, 4);



        //turn to face wall
        encoderTurn(0.175, -80);
        //align so that we are at the center mineral
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {
            encoderDriveINNew(-6, -6, 0.35, 2);

        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT) {
            encoderDriveINNew(4, 4, 0.35, 6);

        }
        robot.disableVision();

    }

}
