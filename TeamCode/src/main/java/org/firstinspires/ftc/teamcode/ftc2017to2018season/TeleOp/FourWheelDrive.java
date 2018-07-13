package org.firstinspires.ftc.teamcode.ftc2017to2018season.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Team Inspiration on 1/21/18.
 */
@TeleOp(name = "Four Wheel Drive")

//This is a basic four-wheel drive program meant for our demo robot.
//This has four motors for drive, one motor for the slides, and two servos for the grabbing arms.
//Please be wary and use basic common sense when looking at this code. For example, don't ask what the
// "slideMotor" is, as it is the slide motor!

//I tried to comment the code as if a sociopath who knows where I live reads it.
//How to working of the normal FTC OpMode works (please don't complain how it's "ineffective and unorganized" and there's some other
// much better alternative or how linux runs things better because this is what we have and we don't have any other alternative)
// So there is the main declaration part. It is when the class begins up until the init() method.
// Then, the init method initializes the hardware. The loop is all the code that runs repetetively.
// The loop is similar to the FRC TeleOpEnabled() method.
//After all of the OpModes are the functions used
// P.S: Sorry if the comments seem condescending or rude

public class FourWheelDrive extends OpMode {

    DcMotor leftWheelMotorFront;
    DcMotor leftWheelMotorBack;
    DcMotor rightWheelMotorFront;
    DcMotor rightWheelMotorBack;
    DcMotor slideMotor;
    Servo glyphServoRight;
    Servo glyphServoLeft;
    public int IVFSM;
    //All the hardware, IVFSM is the baseline value for the slide motor.

    @Override
    public void init() {

        leftWheelMotorFront = hardwareMap.dcMotor.get("leftWheelMotorFront");
        leftWheelMotorBack = hardwareMap.dcMotor.get("leftWheelMotorBack");
        rightWheelMotorFront = hardwareMap.dcMotor.get("rightWheelMotorFront");
        rightWheelMotorBack = hardwareMap.dcMotor.get("rightWheelMotorBack");
        glyphServoRight = hardwareMap.servo.get("glyphServoRight");
        glyphServoLeft = hardwareMap.servo.get("glyphServoLeft");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        IVFSM = slideMotor.getCurrentPosition();

        //declaration of all hardware to the config file

        leftWheelMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftWheelMotorBack.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverses some motors to work normally

        openGlyph();
        //what do YOU think this does??

    }


    @Override
    public void loop() {
       Glyph();
       Drive();
       Slides();
       //Each funciton running is a compilation of all the functions for that major hardware device
    }

    public void Slides(){
        slideMove();
        //A basic function that moves slides up and down
    }
    public void Drive(){
        FourWheelDrive();
        //Common sense should dictate what this function does
    }
    public void Glyph() {
        glyphManipulator();
        //Makes the glyph servos go to to the middle, closed, or open position
        incrementOpen();
        //Incrementally opens the glyphs
        incrementClose();
        //Incrementally closes the glyphs
    }


    public void FourWheelDrive() {

        float leftY_gp1 = (-gamepad1.left_stick_y);
        float rightY_gp1 = (-gamepad1.right_stick_y);
        telemetry.addData("right power input", rightY_gp1);
        telemetry.addData("left power input", leftY_gp1);

            leftWheelMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftWheelMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightWheelMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightWheelMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftWheelMotorFront.setPower(leftY_gp1);
            leftWheelMotorBack.setPower(leftY_gp1);
            rightWheelMotorFront.setPower(rightY_gp1);
            rightWheelMotorBack.setPower(rightY_gp1);
        //Again, common sense _ _
        //                    ___
    }

    public void slideMove() {

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IVFSM = slideMotor.getCurrentPosition();

        if (gamepad2.right_stick_y != 0) {
            slideMotor.setPower(gamepad2.right_stick_y);

        } else {
            slideMotor.setPower(0);
        }

        //Moves the slide according to the value of the joystick. We set it to zero otherwise to stop slipping
    }

    public void glyphManipulator() {


        if (gamepad1.right_bumper&&gamepad1.left_bumper){

            middleGlyph();

        }
        else if (gamepad1.left_bumper) {

            openGlyph();

        } else if (gamepad1.right_bumper) {

            closeGlyph();

        }

    }
    public void wait(int mSec){
        double startTime;
        double endTime;

        startTime = System.currentTimeMillis();
        endTime = startTime+mSec;

        while(endTime >= System.currentTimeMillis()){

            //A sleep function to wait x seconds
        }
    }
    public void incrementOpen(){

        while (gamepad1.x){
            glyphServoLeft.setPosition(glyphServoLeft.getPosition()+0.05);
            glyphServoRight.setPosition(glyphServoRight.getPosition()-0.05);
            wait(300);
        }
    }
    public void incrementClose(){

        while (gamepad1.y) {
            glyphServoLeft.setPosition(glyphServoLeft.getPosition()-0.05);
            glyphServoRight.setPosition(glyphServoRight.getPosition()+0.05);
            wait(300);
        }
    }


    public void openGlyph(){

        glyphServoRight.setPosition(0.5);
        glyphServoLeft.setPosition(0.4);
    }

    public void closeGlyph(){

        glyphServoRight.setPosition(0.75);
        glyphServoLeft.setPosition(0.15);
    }

    public void middleGlyph(){

        glyphServoRight.setPosition(0.65);
        glyphServoLeft.setPosition(0.25);
    }
}
