package org.firstinspires.ftc.teamcode.Main.OpModes.Debug.TestHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;

@Disabled
@Autonomous(name = "Test Encoder Drive")
public class testEncoderDrive extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        setHardwareMap(hardwareMap);
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.liftSlides = hardwareMap.dcMotor.get(constants.LIFT_SLIDES_NAME);

        // Telemetry confirms successful initialization
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        encoderDriveMotor(robot.leftBack, 5, 0.6, 4);
        sleep(2000);
        encoderDriveMotor(robot.leftFront, 5, 0.6, 4);
        sleep(2000);
        encoderDriveMotor(robot.rightBack, 5, 0.6, 4);
        sleep(2000);
        encoderDriveMotor(robot.rightFront, 5, 0.6, 4);
        sleep(500);
        encoderDriveIN(10, 10, 0.4, 6);

    }

}
