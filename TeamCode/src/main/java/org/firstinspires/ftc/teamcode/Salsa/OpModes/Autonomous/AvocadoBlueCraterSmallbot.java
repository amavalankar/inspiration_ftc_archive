package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

@Autonomous(name = "Avocado Blue Crater")
public class AvocadoBlueCraterSmallbot extends SalsaLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initColorSensors();
        // Telemetry confirms successful initialization
        telemetry.addLine("Initialization done ... Ready to start!");
        telemetry.update();

        waitForStart();
        resetEncoderAngle();

        // Align
        alignTape();



    }
}
