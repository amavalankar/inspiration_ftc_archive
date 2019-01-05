package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Disabled
@Autonomous(name = "Turn Until Blue")
public class ColorSensorTest extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

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

        while (opModeIsActive() && !(robot.rightLine.blue() > robot.rightLine.green())) {

//            telemetry.addData("Blue", robot.rightLine.blue());
//            telemetry.addData("Red", robot.rightLine.red());
//            telemetry.addData("Green", robot.rightLine.green());
//            telemetry.update();
            doEncoderTurn(0.25, 10);

        }


    }

}