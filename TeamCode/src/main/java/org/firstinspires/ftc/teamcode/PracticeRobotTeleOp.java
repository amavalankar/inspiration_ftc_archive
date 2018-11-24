package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Practice Mecanum")
public class PracticeRobotTeleOp extends OpMode{
    public DcMotor leftWheelFront;
    public DcMotor leftWheelBack;
    public DcMotor rightWheelFront;
    public DcMotor rightWheelBack;

    public DcMotor hanger;

    public void init (){
        leftWheelFront = hardwareMap.dcMotor.get("leftWheelFront");
        leftWheelBack = hardwareMap.dcMotor.get("leftWheelBack");
        rightWheelFront = hardwareMap.dcMotor.get("rightWheelFront");
        rightWheelBack = hardwareMap.dcMotor.get("rightWheelBack");

        telemetry.addLine("Press Start!");
        telemetry.update();
    }

    public void loop (){
        FourWheelDrive();
        dpad();
        hanger();
    }

    public void FourWheelDrive (){
        double leftY_gp1 = (-gamepad1.left_stick_y);
        double rightY_gp1 = (gamepad1.right_stick_y);

        leftWheelFront.setPower(leftY_gp1);
        leftWheelBack.setPower(leftY_gp1);
        rightWheelFront.setPower(rightY_gp1);
        rightWheelBack.setPower(rightY_gp1);
    }

    public void dpad(){

        int leftSide = -1;

        if (gamepad1.dpad_up){
            leftWheelFront.setPower(0.5 * leftSide);
            leftWheelBack.setPower(0.5 * leftSide);
            rightWheelFront.setPower(0.5);
            rightWheelBack.setPower(0.5);
        }
        else if (gamepad1.dpad_down){
            leftWheelFront.setPower(-0.5 * leftSide);
            leftWheelBack.setPower(-0.5 * leftSide);
            rightWheelFront.setPower(-0.5);
            rightWheelBack.setPower(-0.5);
        }
        else if (gamepad1.dpad_right){
            leftWheelFront.setPower(1 * leftSide);
            leftWheelBack.setPower(-1 * leftSide);
            rightWheelFront.setPower(-1);
            rightWheelBack.setPower(1);
        }
        else if (gamepad1.dpad_left){
            leftWheelFront.setPower(-1 * leftSide);
            leftWheelBack.setPower(1 * leftSide);
            rightWheelFront.setPower(1);
            rightWheelBack.setPower(-1);
        }
    }


    public void hanger (){
        double leftY_gp2 = (-gamepad2.left_stick_y);

        hanger.setPower(leftY_gp2);
    }
}
