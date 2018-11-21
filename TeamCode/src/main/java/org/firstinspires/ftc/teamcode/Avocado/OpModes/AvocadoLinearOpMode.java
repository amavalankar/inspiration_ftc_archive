package org.firstinspires.ftc.teamcode.Avocado.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Avocado.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Avocado.Hardware.Robot;

public abstract class AvocadoLinearOpMode extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Robot robot = new Robot();
    Constants constants;

    public void encoderDrive(double speed, double leftCM, double rightCM, double timeoutS) {

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.topLeftMotor.getCurrentPosition() + (int) (leftCM * constants.TICKS_PER_CM);
            newLeftTarget = robot.bottomLeftMotor.getCurrentPosition() + (int) (leftCM * constants.TICKS_PER_CM);
            newRightTarget = robot.topRightMotor.getCurrentPosition() + (int) (rightCM * constants.TICKS_PER_CM);
            newRightTarget = robot.bottomRightMotor.getCurrentPosition() + (int) (rightCM * constants.TICKS_PER_CM);

            robot.topLeftMotor.setTargetPosition(newLeftTarget);
            robot.bottomLeftMotor.setTargetPosition(newLeftTarget);
            robot.topRightMotor.setTargetPosition(newRightTarget);
            robot.bottomRightMotor.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.topLeftMotor.setPower(Math.abs(speed));
            robot.topRightMotor.setPower(Math.abs(speed));
            robot.bottomLeftMotor.setPower(Math.abs(speed));
            robot.topRightMotor.setPower(Math.abs(speed));

            //If you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while ((runtime.seconds() < timeoutS) && (robot.topLeftMotor.isBusy() || robot.bottomLeftMotor.isBusy() || robot.topRightMotor.isBusy() || robot.bottomRightMotor.isBusy())) {

                // Telemetry
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.topLeftMotor.getCurrentPosition(),
                        robot.topRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop the motors
            robot.topLeftMotor.setPower(0);
            robot.bottomLeftMotor.setPower(0);
            robot.topRightMotor.setPower(0);
            robot.bottomRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
