package org.firstinspires.ftc.teamcode.Salsa.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Salsa.OpModes.SalsaLinearOpMode;

@Autonomous(name = "DeHang and Sample")
public class DeHang_and_Sample extends SalsaLinearOpMode {

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
//        alignTape();


        while(robot.leftLineDist.getDistance(DistanceUnit.INCH) > 2) {

            robot.liftSlides.setPower(1);

        }

        robot.liftSlides.setPower(0);

        encoderDriveIN(-4, -4, 0.7, 3);




    }
}
