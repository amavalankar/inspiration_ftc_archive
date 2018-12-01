package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

@Autonomous(name = "Avocado Blue Crater Drive")
public class AutonomousDriveBlueCrater extends SalsaLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.initDistanceSensors();
        // Telemetry confirms successful initialization
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        encoderTurn(1, -90);
        while(robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 3) {
            encoderDriveCM(1, 1, 1, 3);
        }
        encoderTurn(1, -45);
        while(robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18) {
            encoderDriveCM(1, 1, 1, 3);
        }

        robot.markerDepositer.setPosition(1);

    }
}