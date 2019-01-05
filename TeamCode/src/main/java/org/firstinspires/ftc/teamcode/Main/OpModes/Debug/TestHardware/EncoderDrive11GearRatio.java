package org.firstinspires.ftc.teamcode.Main.OpModes.Debug.TestHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Disabled
@Autonomous(name = "Encoder Drive GR 1:1", group = "Avocado")
public class EncoderDrive11GearRatio extends ExtendedLinearOpMode {


    @Override
    public void runOpMode() {

        // Initialize
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

        // Methods
        encoderDriveIN(24, 24, 0.25, 10);

    }
}


