package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Avocado TeleOp", group = "Avocado")
public class EncoderDrive4872GearRatio extends ExtendedLinearOpMode {


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
            encoderDriveINNew(12, 12, 0.5, 10);

        }
    }


