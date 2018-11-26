package org.firstinspires.ftc.teamcode.Salsa.OpModes;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Salsa.Constants;
import org.firstinspires.ftc.teamcode.Salsa.Hardware.Robot;

/**
 * Created by adityamavalankar on 11/19/18.
 */

public abstract class SalsaOpMode extends OpMode {

    /**
     * This is a modified version of the {OpMode} class, with all of the functions
     * meant for TeleOp, without having extra work to make them, and the remote-controlled OpMode
     * to both work
     */

    public Robot robot = new Robot();

    public Constants constants;

    /**
     *
     * @param leftJoystick
     * @param rightJoystick
     */
    public void drive(double leftJoystick, double rightJoystick) {

        robot.leftBack.setPower(leftJoystick);
        robot.leftFront.setPower(leftJoystick);
        robot.rightFront.setPower(rightJoystick);
        robot.rightBack.setPower(rightJoystick);

    }

    /**
     * For mecanumDrive(), we use the dpad to go left/right. We move the alternate motors forward/reverse.
     * @param dpad_left
     * @param dpad_right
     */

    public void mecanumDrive(boolean dpad_left, boolean dpad_right) {

        if (dpad_left) {
            robot.leftBack.setPower(-1);
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(-1);
            robot.rightBack.setPower(1);
        }
        else if (dpad_right) {
            robot.leftBack.setPower(1);
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(1);
            robot.rightBack.setPower(-1);
        }
    }

    public void collect(boolean right_bumper, boolean left_bumper) {
        if (right_bumper) {
            robot.collector.setPower(1);
        }
        
        else if (left_bumper) {
            robot.collector.setPower(-1);
        } else {

            robot.collector.setPower(0);

        }
    }

    public void extend(double gp2_leftY) {
        robot.extension.setPower(gp2_leftY);
    }

    public void tilt(double gp2_rightY) {
        robot.tiltMotor.setPower(gp2_rightY);
    }

    public void strafe(boolean left, boolean right, boolean up, boolean down, double speed) {
        // Move robot in the direction corresponding to the DPad
        if (left) {

            robot.leftFront.setPower(1);
            robot.leftBack.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.rightBack.setPower(1);

        } else if (right) {

            robot.leftFront.setPower(-1);
            robot.leftBack.setPower(1);
            robot.rightFront.setPower(1);
            robot.rightBack.setPower(-1);

        } else if (up) {

            robot.leftFront.setPower(-speed);
            robot.leftBack.setPower(-speed);
            robot.rightFront.setPower(-speed);
            robot.rightBack.setPower(-speed);


        } else if (down) {

            robot.leftFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.rightBack.setPower(speed);

        }

    }

    public void hang(boolean up, boolean down) {

        if (up) {

            robot.liftSlides.setPower(1);

        } else if (down) {

            robot.liftSlides.setPower(-1);

        } else {

            robot.liftSlides.setPower(0);

        }
    }

}
