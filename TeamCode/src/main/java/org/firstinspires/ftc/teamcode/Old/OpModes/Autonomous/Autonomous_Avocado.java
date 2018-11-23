package org.firstinspires.ftc.teamcode.Old.OpModes.Autonomous;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Old.Hardware.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Old.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Old.OpModes.AvocadoLinearOpMode;

@Autonomous(name="Avocado Autonomous")
@Disabled
public class Autonomous_Avocado extends AvocadoLinearOpMode {

    Robot robot = new Robot();
    Constants constants;

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        lower();

    }
}
