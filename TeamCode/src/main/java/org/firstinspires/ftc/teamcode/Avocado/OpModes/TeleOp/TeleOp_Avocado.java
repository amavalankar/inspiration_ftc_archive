package org.firstinspires.ftc.teamcode.Avocado.OpModes.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Avocado.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Avocado.OpModes.AvocadoOpMode;

/* ----------------------------------------------------------------------------------------------- */

/** Future master TeleOp file - NOT TESTED */

/* ----------------------------------------------------------------------------------------------- */

@TeleOp(name = "Avacado TeleOp")


public class TeleOp_Avocado extends AvocadoOpMode {

    Robot robot = new Robot();


    public void loop() {

        TankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        lift_a(gamepad2.left_stick_y);
        strafe(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down, 0.5);

    }

    public void init() {
        robot.init(hardwareMap);
    }

}
