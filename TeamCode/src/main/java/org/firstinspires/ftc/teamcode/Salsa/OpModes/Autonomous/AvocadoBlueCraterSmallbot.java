package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

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
