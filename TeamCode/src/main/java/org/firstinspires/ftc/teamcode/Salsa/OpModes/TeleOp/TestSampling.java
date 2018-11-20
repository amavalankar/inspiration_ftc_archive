package org.firstinspires.ftc.teamcode.Salsa.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Salsa.Vision.CameraCropAngle;
import org.firstinspires.ftc.teamcode.Salsa.Vision.SamplingDetector;

/**
 * Created by adityamavalankar on 11/12/18.
 */

@TeleOp(name="Sampling Test Doge CV")
public class TestSampling extends OpMode {

    SamplingDetector detector = null;

    @Override
    public void init() {
        detector = new SamplingDetector();
        detector.initVision(hardwareMap, CameraCropAngle.RIGHT);
        detector.enableVision();
    }

    @Override
    public void loop() {
        telemetry.addData("Order", detector.getSamplingOrder());
        telemetry.update();
    }

    @Override
    public void stop() {
        detector.disableVision();
    }
}
