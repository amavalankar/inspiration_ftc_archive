package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;

@Autonomous(name = "Avocado Blue Crater Drive")
public class AutonomousDriveBlueCrater extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        robot.initDistanceSensors();
        robot.markerDepositer = hardwareMap.servo.get("markerDepositer");
        // Telemetry confirms successful initialization
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        setPower(0.25);
        while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 20) && opModeIsActive()) {
        }
        setPower(0);

        encoderTurn(0.3, -40);
        sleep(200);

        setPower(0.25);
        while((robot.wallAlignFront.getDistance(DistanceUnit.INCH) > 18)&& opModeIsActive()) {
        }
        setPower(0);

        encoderTurn(0.4, -10);

        robot.markerDepositer.setPosition(0);
        sleep(1500);

        encoderDriveINNew(-5, -5, 0.3, 4);

        encoderTurn(0.4, -40);
        
        encoderDriveINNew(-30, -30, 0.3, 4);

        robot.markerDepositer.setPosition(0.2);

        encoderTurn(0.4, 95);

        encoderDriveINNew(60, 60, 0.5, 8);
    }
}
