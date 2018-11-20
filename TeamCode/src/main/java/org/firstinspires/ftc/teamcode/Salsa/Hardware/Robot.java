package org.firstinspires.ftc.teamcode.Salsa.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Salsa.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Salsa.Vision.SamplingDetector;

/**
 * Created by adityamavalankar on 11/4/18.
 */

public class Robot {

    public OpMode opMode;
    private Constants constants = new Constants();

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public BNO055IMU imu;
    public ColorSensor leftLine;
    public ColorSensor rightLine;
    public WebcamName webcamFront;
    public Orientation angles;
    public Servo leftMineral;
    public Servo rightMineral;
    public CRServo mineralFeeder;
    public Servo depositerRotate;
    public Servo depositerDump;
    public DistanceSensor groundDistance;

    public DcMotor mineralShooter;
    public DcMotor craterSlides;
    public DcMotor intakeMotor;
    public DcMotor liftSlides;

    public Servo intakeLifter;
    public Servo markerDepositer;

    public HardwareMap ahwmap;
    public SamplingDetector samplingDetector = new SamplingDetector();

    public void setHardwareMap(HardwareMap hwMap) {
        ahwmap = hwMap;
    }

    public void initDrivetrain() {

        //Drivetrain
        leftFront = ahwmap.dcMotor.get(constants.LEFT_FRONT_NAME);
        leftBack = ahwmap.dcMotor.get(constants.LEFT_BACK_NAME);
        rightFront = ahwmap.dcMotor.get(constants.RIGHT_FRONT_NAME);
        rightBack = ahwmap.dcMotor.get(constants.RIGHT_BACK_NAME);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void initMotors() {

        mineralShooter = ahwmap.dcMotor.get(constants.MINERAL_SHOOTER_NAME);
        craterSlides = ahwmap.dcMotor.get(constants.CRATER_SLIDES_NAME);
        intakeMotor = ahwmap.dcMotor.get(constants.INTAKE_MOTOR_NAME);
        liftSlides = ahwmap.dcMotor.get(constants.LIFT_SLIDES_NAME);

    }

    public void initSensors() {

        //Sensors
        imu = ahwmap.get(BNO055IMU.class, constants.GYRO_NAME);
        leftLine = ahwmap.get(ColorSensor.class, constants.LEFT_COLOR_NAME);
        rightLine = ahwmap.get(ColorSensor.class, constants.RIGHT_COLOR_NAME);
        groundDistance = ahwmap.get(DistanceSensor.class, constants.GROUND_DISTANCE_SENSOR_NAME);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void initWebcam() {

        //Webcam
        webcamFront = ahwmap.get(WebcamName.class, constants.WEBCAM_FRONT_NAME);

    }

    public void initServos() {

        //Servos
        intakeLifter = ahwmap.servo.get(constants.INTAKE_LIFTER_NAME);
        markerDepositer = ahwmap.servo.get(constants.MARKER_DEPOSITER_NAME);
        leftMineral = ahwmap.servo.get(constants.LEFT_MINERAL_NAME);
        rightMineral = ahwmap.servo.get(constants.RIGHT_MINERAL_NAME);
        mineralFeeder = ahwmap.crservo.get(constants.MINERAL_FEEDER_NAME);
        depositerRotate = ahwmap.servo.get(constants.DEPOSITER_ROTATE_NAME);
        depositerDump = ahwmap.servo.get(constants.DEPOSITER_DUMP_NAME);

    }
}
