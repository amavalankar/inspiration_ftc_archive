package org.firstinspires.ftc.teamcode.Main.OpModes.Debug.TestHardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Distance Sensor Test")
public class DistanceSensorTest extends ExtendedLinearOpMode {



    @Override
    public void runOpMode() {

        setHardwareMap(hardwareMap);

        robot.wallAlignFront = hardwareMap.get(DistanceSensor.class, constants.WALL_ALIGN_FRONT_NAME);

        // Telemetry confirms successful initialization. It's delayed to let everything load
        sleep(3000);
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();
    while (opModeIsActive()) {
        telemetry.addData("Distance:", robot.wallAlignFront.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
    }
}
