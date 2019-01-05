package org.firstinspires.ftc.teamcode.Main.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Main.Constants;
import org.firstinspires.ftc.teamcode.Main.Hardware.Robot;

/**
 * Created by adityamavalankar on 11/19/18.
 */

public abstract class ExtendedOpMode extends OpMode {

    /**
     * This is a modified version of the {OpMode} class, with all of the functions
     * meant for TeleOpMain, without having extra work to make them, and the remote-controlled OpMode
     * to both work
     */

    /**
     * We create a Robot object for the class above to instantiate. This makes it easier on the classes
     */
    public Robot robot = new Robot();

    public Constants constants;

    /**
     *
     * @param leftJoystick
     * @param rightJoystick
     */
    public void drive(double leftJoystick, double rightJoystick) {

        robot.leftBack.setPower(-leftJoystick);
        robot.leftFront.setPower(-leftJoystick);
        robot.rightFront.setPower(-rightJoystick);
        robot.rightBack.setPower(-rightJoystick);

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

    /**
     * A void to collect the minerals. We just move one motor forward/backwards
     * @param right_bumper
     * @param left_bumper
     */
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

    public void setPower(double left_power, double right_power) {
        robot.leftFront.setPower(left_power);
        robot.leftBack.setPower(left_power);
        robot.rightFront.setPower(right_power);
        robot.rightBack.setPower(right_power);

    }

    /**
     * 
     * @param gp2_leftY
     */
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

    public void lockServos(boolean d1_x, boolean d1_y) {

        if (d1_x) {
            //make it closed
            robot.leftLockServo.setPosition(constants.LEFT_LOCK_SERVO_CLOSED_POS);
            robot.rightLockServo.setPosition(constants.RIGHT_LOCK_SERVO_CLOSED_POS);
        }
        else if (d1_y) {
            //make it open
            robot.leftLockServo.setPosition(constants.LEFT_LOCK_SERVO_OPEN_POS);
            robot.rightLockServo.setPosition(constants.RIGHT_LOCK_SERVO_OPEN_POS);
        }
    }

    public void dumperServo(boolean d2_x, boolean d2_y){

        if(d2_x){
            robot.dumperServo.setPosition(0.5);
        }
        if (d2_y){
            robot.dumperServo.setPosition(0);
        }
        //Open and closing dumper
    }

    /**
     * This is a void taken from {LinearOpMode} to have a sleep function for x milliseconds
     * To learn more about it, refer to the original JavaDoc in the original {LinearOpMode} class made by FIRST
     * This sleep void is special than others because it can get interrupted without crashing
     * @param milliseconds Is how ling it will sleep for
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
