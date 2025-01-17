package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.ExtendedOpMode;

@Disabled
@TeleOp(name = "League Meet One TeleOp", group = "Old")
public class LeagueMeetOneTeleOp extends ExtendedOpMode {


    @Override
    public void init() {

        // Initiate all the hardwareMap names
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initHanger();
        robot.markerDepositer = hardwareMap.servo.get("markerDepositer");
        sleep(50);
        robot.markerDepositer.setPosition(0.2);

    }

    @Override
    public void loop() {
        // Run TeleOP methods
        drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        hang(gamepad2.dpad_up, gamepad2.dpad_down);
        strafe(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, 0.5);

    }
}
