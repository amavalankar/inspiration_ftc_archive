package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedLinearOpMode;
import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

@Autonomous(name = "Just Dehang")
public class JustDehang extends ExtendedLinearOpMode {

    @Override
    public void runOpMode() {

        // initiate
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
        resetEncoderAngle();

        // Dehang
        moveActuator(-4, 6);
        //for some reason life is falling apart my life is just falling apart why does this always happen
        // to me oh my god they did surgery on a grape

        //intially, the thing was working properly, but due to some unknown reason, it is not working, it hits the top and
        //doesn't stop.'
        sleep(1500);
        // Unhook
        resetEncoderAngle();
        //encoderDriveINNew(-5, -5, 0.25, 3);
        sleep(100);
        moveActuator(4, 2);



    }

}