package org.firstinspires.ftc.teamcode.Main.OpModes.Debug.StrafeTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Strafe -0.6 Offset", group = "Avocado")
public class Strafe06 extends ExtendedLinearOpMode {


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
        encoderStrafeOffset(30, 1, 0, 0.6);
        encoderStrafeOffset(-30, 1, 0, 0.6);

    }
}


