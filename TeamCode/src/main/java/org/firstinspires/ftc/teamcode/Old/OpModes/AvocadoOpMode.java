package org.firstinspires.ftc.teamcode.Old.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Old.Hardware.Robot;


public abstract class AvocadoOpMode extends OpMode {
    
    Robot robot = new Robot();
    // posleft is used to correct the direction of the motors on the left side of the robot
    byte posleft = -1;

    public void TankDrive(float leftdrive, float rightdrive) {

        // Passes controller values to the motors
        robot.topLeftMotor.setPower(-leftdrive);
        robot.bottomLeftMotor.setPower(-leftdrive);
        robot.topRightMotor.setPower(rightdrive);
        robot.bottomRightMotor.setPower(rightdrive);

    }

    public void lift_a(float lift) {

        // Pass controller value into the lift motor
        robot.hanger.setPower(-lift);

    }

    public void lift_b(boolean up, boolean down, double speed) {

        // If up is pressed move motor in a positive direction. If down is pressed, move it in a negative direction.
        if (up) {

            robot.hanger.setPower(speed);

        } else if (down) {

            robot.hanger.setPower(speed);

        }

    }

    public void strafe(boolean left, boolean right, boolean up, boolean down, double speed) {
        // Move robot in the direction corresponding to the DPad
        if (left) {

            robot.topLeftMotor.setPower(1 * posleft);
            robot.bottomLeftMotor.setPower(-1 * posleft);
            robot.topRightMotor.setPower(-1);
            robot.bottomRightMotor.setPower(1);

        } else if (right) {

            robot.topLeftMotor.setPower(-1 * posleft);
            robot.bottomLeftMotor.setPower(1 * posleft);
            robot.topRightMotor.setPower(1);
            robot.bottomRightMotor.setPower(-1);

        } else if (up) {

            robot.topLeftMotor.setPower(speed * posleft);
            robot.bottomLeftMotor.setPower(speed * posleft);
            robot.topRightMotor.setPower(speed);
            robot.bottomRightMotor.setPower(speed);


        } else if (down) {

            robot.topLeftMotor.setPower(-speed * posleft);
            robot.bottomLeftMotor.setPower(-speed * posleft);
            robot.topRightMotor.setPower(-speed);
            robot.bottomRightMotor.setPower(-speed);

        }

    }
}
