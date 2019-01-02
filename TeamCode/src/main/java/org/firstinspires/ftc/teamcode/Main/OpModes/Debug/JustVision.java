package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Just Vision")
public class JustVision extends ExtendedLinearOpMode {

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

        //save sampling order of minerals to this variable

        SamplingOrderDetector.GoldLocation goldLocation = robot.getSamplingOrder();
        sleep(400);

        while (opModeIsActive()) {
            telemetry.addData("Current Orientation is", robot.getSamplingOrder());
            telemetry.update();
        }

        robot.disableVision();
    }
}