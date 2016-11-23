package org.firstinspires.ftc.teamcode.Jack;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by inspirationteam on 11/10/2016.
 */
@TeleOp(name = "#11183: Jack TeleOp Competition", group = "Robot")
public class JackCompetitionTeleOp extends OpMode {

    //---------------------------------------------------------------------------------------------
/*
   Define the actuators we use in the robot here
*/
    DcMotor leftWheelMotorFront;
    DcMotor leftWheelMotorBack;
    DcMotor rightWheelMotorFront;
    DcMotor rightWheelMotorBack;
    DcMotor ballCollectorMotor;
    DcMotor ballShooterMotor;

// ----------------------------------------------------------------------------------------------

    /*
        Get references to the hardware installed on the robot and name them here
    */
    @Override
    public void init() {
        leftWheelMotorFront = hardwareMap.dcMotor.get("leftWheelMotorFront");
        leftWheelMotorBack = hardwareMap.dcMotor.get("leftWheelMotorBack");
        rightWheelMotorFront = hardwareMap.dcMotor.get("rightWheelMotorFront");
        rightWheelMotorBack = hardwareMap.dcMotor.get("rightWheelMotorBack");
        ballCollectorMotor = hardwareMap.dcMotor.get("ballCollectorMotor");
        ballShooterMotor = hardwareMap.dcMotor.get("ballShooterMotor");

        /* lets reverse the direction of the right wheel motor*/
        rightWheelMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightWheelMotorBack.setDirection(DcMotor.Direction.REVERSE);
    }
//---------------------------------------------------------------------------------------------

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

//---------------------------------------------------------------------------------------------

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }
//---------------------------------------------------------------------------------------------
/*
Main code loop goes here
 */
    @Override
<<<<<<< HEAD
    public void loop() {
=======
    public void loop(){
        // read the gamepad values and put into variables
>>>>>>> d1af4182201c10d2e493898c6c5a20866d31a98f
        float leftY_gp1 = -gamepad1.left_stick_y;
        float rightY_gp1 = -gamepad1.right_stick_y;

        //run the motors by setting power to the motors with the game pad values
        leftWheelMotorFront.setPower(leftY_gp1);
        leftWheelMotorBack.setPower(leftY_gp1);
        rightWheelMotorFront.setPower(rightY_gp1);
        rightWheelMotorBack.setPower(rightY_gp1);
    }
<<<<<<< HEAD

    public void CollectBalls() {
=======
//---------------------------------------------------------------------------------------------


    public void CollectBalls(){
>>>>>>> d1af4182201c10d2e493898c6c5a20866d31a98f
        boolean intake = gamepad1.right_bumper;
        boolean outtake = gamepad1.left_bumper;

        if (intake) {
            ballCollectorMotor.setPower(1);
        } else if (outtake) {
            ballCollectorMotor.setPower(-1);
        } else {
            ballCollectorMotor.setPower(0);
        }
<<<<<<< HEAD
    }
}
=======

//---------------------------------------------------------------------------------------------
>>>>>>> d1af4182201c10d2e493898c6c5a20866d31a98f
    /*
     * Code to run ONCE after the driver hits STOP
     */
    /*@Override
    public void stop() {
    }
   */

<<<<<<< HEAD
=======
}
//---------------------------------------------------------------------------------------------
>>>>>>> d1af4182201c10d2e493898c6c5a20866d31a98f