package org.firstinspires.ftc.teamcode.Avocado.OpModes.Autonomous;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Avocado.Hardware.Robot;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Avocado.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Avocado.OpModes.AvocadoLinearOpMode;
import org.firstinspires.ftc.teamcode.Avocado.OpModes.AvocadoOpMode;

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
