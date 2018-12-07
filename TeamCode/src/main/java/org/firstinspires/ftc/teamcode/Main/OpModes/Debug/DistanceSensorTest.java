package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedOpMode;

@TeleOp(name = "Avocado TeleOp Practice")
public class DistanceSensorTest extends ExtendedOpMode {

    @Override
    public void init() {



    }

    @Override
    public void loop() {

        telemetry.addData("Distance:", robot.wallAlignFront.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }
}
