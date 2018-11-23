package org.firstinspires.ftc.teamcode.Old.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Old.Hardware.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

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

        /**
         * The REV Robotics Touch Sensor
         * is treated as a digital channel.  It is HIGH if the button is unpressed.
         * It pulls LOW if the button is pressed.
         *
         * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
         * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
         * The lower (first) pin stays unconnected.*
         */



        public void lower() {
            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)robot.sensorRange;

            waitForStart();
            while(opModeIsActive()) {
                // generic DistanceSensor methods.
                telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );
                telemetry.addData("range", String.format("%.01f mm", robot.sensorRange.getDistance(DistanceUnit.MM)));
                telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f m", robot.sensorRange.getDistance(DistanceUnit.METER)));
                telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));

                // Rev2mDistanceSensor specific methods.
                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                telemetry.update();
            }
        }
    }
