package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.ExtendedOpMode;

@TeleOp(name = "League Meet Three TeleOp", group = "Final")
public class LeagueMeetThreeTeleOp extends ExtendedOpMode {

    @Override
    public void init() {

        // Initiate all the hardwareMap names
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initTiltingMechanism();
        robot.initHanger();
        robot.initServo();

    }

    @Override
    public void init_loop() {
        telemetry.addLine("Robot Initialized ... Ready to go!");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Run TeleOP methods
        drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        collect(gamepad1.right_bumper, gamepad1.left_bumper);
        extend(gamepad2.left_stick_y);
        tilt(gamepad2.right_stick_y);
        hang(gamepad2.dpad_up, gamepad2.dpad_down);
        strafe(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, 0.5);
//      lockServos(gamepad1.x, gamepad1.y);  // <-- This line of code tries to reference hardware that can't be accessed.
        dumperServo(gamepad2.left_bumper, gamepad2.right_bumper);


        telemetry.addLine("In Loop");
        telemetry.update();
    }
}

