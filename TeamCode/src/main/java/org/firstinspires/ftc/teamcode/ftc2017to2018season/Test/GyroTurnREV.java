package org.firstinspires.ftc.teamcode.ftc2017to2018season.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc2017to2018season.Autonomous.Autonomous_General_George;

/**
 * Created by Inspiration Team on 1/21/2018.
 */
//@Autonomous(name = "gyroTurnREV_final")
public class GyroTurnREV extends Autonomous_General_George {

    @Override
    public void runOpMode() {
        initiate(false);

        waitForStart();
        gyroTurnREV(0.5,90);
    }

}