package org.firstinspires.ftc.teamcode.Salsa.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Salsa.Constants;

/**
 * Created by adityamavalankar on 11/4/18.
 */

public class Robot {

    public HardwareMap hwmap = null;
    public Constants constants = new Constants();

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public BNO055IMU imu = null;
    public ColorSensor leftLine = null;
    public ColorSensor rightLine = null;
    public WebcamName webcamFront = null;

    public void initDrivetrain(HardwareMap inputMap) {

        hwmap = inputMap;

        //Drivetrain
        leftFront = hwmap.get(DcMotor.class, constants.LEFT_FRONT_NAME);
        leftBack = hwmap.get(DcMotor.class, constants.LEFT_BACK_NAME);
        rightFront = hwmap.get(DcMotor.class, constants.RIGHT_FRONT_NAME);
        rightBack = hwmap.get(DcMotor.class, constants.RIGHT_BACK_NAME);
    }

    public void initSensors(HardwareMap inputMap) {

        hwmap = inputMap;

        //Sensors
        imu = hwmap.get(BNO055IMU.class, constants.GYRO_NAME);

    }

    public void initWebcam(HardwareMap inputMap) {

        hwmap = inputMap;

        //Webcam
        webcamFront = hwmap.get(WebcamName.class, constants.WEBCAM_FRONT_NAME);

    }
}