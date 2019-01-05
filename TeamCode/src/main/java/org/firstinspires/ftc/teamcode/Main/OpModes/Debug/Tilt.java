package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Disabled
@Autonomous(name = "Tilt Test")
public class Tilt extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        setHardwareMap(hardwareMap);
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.liftSlides = hardwareMap.dcMotor.get(constants.LIFT_SLIDES_NAME);
        robot.initVision(CameraCropAngle.LEFT);
        robot.initServo();
        robot.enableVision();
        robot.initTiltingMechanism(); // oh and noah set something on fire
        // Telemetry confirms successful initialization. It's delayed to let everything load
        sleep(3000);
        telemetry.addLine("Initialization tdone ... Ready to start!");
        telemetry.update();

        tiltMarker(1, -0.5);
        tiltMarker(1, 0.5);

        waitForStart();


    }

}