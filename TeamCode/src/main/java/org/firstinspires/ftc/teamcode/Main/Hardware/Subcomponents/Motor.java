package org.firstinspires.ftc.teamcode.Main.Hardware.Subcomponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by adityamavalankar on 11/5/18.
 */

public abstract class Motor implements DcMotor {

    public DcMotor dcm;
    public HardwareMap hwmap;

    public void init(String hardwareName, HardwareMap inputMap) {

        hwmap = inputMap;

        dcm = hwmap.get(DcMotor.class, hardwareName);
    }

}
