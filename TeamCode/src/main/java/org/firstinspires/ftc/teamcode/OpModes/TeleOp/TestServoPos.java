package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.ExtendedOpMode;

@TeleOp(name = "Test Servo Position`", group = "Test")
public class TestServoPos extends ExtendedOpMode {

    @Override
    public void init() {

        // Initiate all the hardwareMap names
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        robot.initTiltingMechanism();
        robot.initHanger();
        robot.initServo();
        robot.initDistanceSensors();

    }

    @Override
    public void init_loop() {
        telemetry.addLine("Robot Initialized ... Ready to go!");
        telemetry.update();
        robot.dumperServo.setPosition(0.5);
    }

    @Override
    public void loop() {

        double current_servo_pos = robot.dumperServo.getPosition();

        if (gamepad1.left_bumper) {
            robot.dumperServo.setPosition(current_servo_pos - 0.05);
            sleep(250);

        } else if(gamepad1.right_bumper) {
            robot.dumperServo.setPosition(current_servo_pos + 0.05);
            sleep(250);

        }

        telemetry.addData("Current servo position", robot.dumperServo.getPosition());
        //telemetry.update();

        telemetry.addLine("In Loop");
        telemetry.update();
    }
}

