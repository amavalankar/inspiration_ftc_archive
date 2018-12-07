package org.firstinspires.ftc.teamcode.Main.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.OpModes.ExtendedOpMode;

/**
 * Created by adityamavalankar on 12/2/18.
 */

@Disabled
@TeleOp(name = "Test Individual Motor", group = "Debug")
public class testIndividualMotor extends ExtendedOpMode {

    DcMotor motorToRun;

    @Override
    public void init() {
        robot.setHardwareMap(hardwareMap);
        robot.initDrivetrain();
        motorToRun = robot.leftFront;

        telemetry.addLine("Ready to go!");
        telemetry.update();
    }

    @Override
    public void loop() {
        selectMotor();
        runMotor();
    }

    void selectMotor() {
        if (gamepad1.a) {
            //lb
            motorToRun = robot.leftBack;
        }
        else if (gamepad1.b) {
            //rb
            motorToRun = robot.rightBack;
        }
        else if (gamepad1.x) {
            //rf
            motorToRun = robot.rightFront;
        }
        else if (gamepad1.y) {
            //lf
            motorToRun = robot.leftFront;
        }
    }

    void runMotor() {
        motorToRun.setPower(gamepad1.left_stick_y);
    }
}
