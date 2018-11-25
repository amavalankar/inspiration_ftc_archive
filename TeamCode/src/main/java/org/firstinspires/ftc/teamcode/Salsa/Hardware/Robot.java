package org.firstinspires.ftc.teamcode.Salsa.Hardware;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.teamcode.Salsa.Vision.CameraCropAngle;
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
    public DcMotor extension;
    public DcMotor tiltMotor;
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

    public DcMotor collector;
    public Servo intakeLifter;
    public Servo markerDepositer;

    public HardwareMap ahwmap;
    public SamplingOrderDetector detector = new SamplingOrderDetector();
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


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

    /*
    public void initDrivetrainAvocado() {

        //Drivetrain
        leftFront = ahwmap.dcMotor.get("topLeftMotor");
        leftBack = ahwmap.dcMotor.get("bottomLeftMotor");
        rightFront = ahwmap.dcMotor.get("topRightMotor");
        rightBack = ahwmap.dcMotor.get("bottomRightMotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    */

    public void initTiltingMechanism() {

        extension = ahwmap.dcMotor.get(constants.EXTENSION_NAME);
        tiltMotor = ahwmap.dcMotor.get(constants.TILT_MOTOR_NAME);
        collector = ahwmap.dcMotor.get(constants.COLLECTOR_NAME);

    }

    public void initHanger() {

        liftSlides = ahwmap.dcMotor.get(constants.LIFT_SLIDES_NAME);

    }

    public void initSensors() {

        //Sensors
        leftLine = ahwmap.get(ColorSensor.class, constants.LEFT_COLOR_NAME);
        rightLine = ahwmap.get(ColorSensor.class, constants.RIGHT_COLOR_NAME);
        groundDistance = ahwmap.get(DistanceSensor.class, constants.GROUND_DISTANCE_SENSOR_NAME);

    }

    public void initGyro() {

        parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = ahwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void resetGyro() {

        imu.initialize(parameters);
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

    public void initVision() {

        initVision(constants.CAMERA_AIM_DIRECTION);

    }

    public void initVision(CameraCropAngle cropAngle) {

        if (cropAngle == CameraCropAngle.LEFT) {

            detector.positionCamRight = false;
            detector.positionCamLeft = true;
        }
        else if (cropAngle == CameraCropAngle.RIGHT) {

            detector.positionCamRight = true;
            detector.positionCamLeft = false;
        }
        else if (cropAngle == CameraCropAngle.NO_CROP) {

            detector.positionCamRight = false;
            detector.positionCamLeft = false;
        }
        else {

            detector.positionCamRight = false;
            detector.positionCamLeft = false;
        }


        detector.init(ahwmap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        detector.useDefaults(); // Set detector to use default settings


        detector.downscale = 0.6; // How much to downscale the input frames

        // Optional tuning

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring

        detector.maxAreaScorer.weight = 0.001;


        detector.ratioScorer.weight = 10;

        detector.ratioScorer.perfectRatio = 1.0;
    }

    public void enableVision() {

        detector.enable();
    }

    public SamplingOrderDetector.GoldLocation getSamplingOrder() {

        SamplingOrderDetector.GoldLocation order;
        order =
                detector.getCurrentOrder();

        return order;
    }

    public SamplingOrderDetector.GoldLocation getLastOrder() {

        SamplingOrderDetector.GoldLocation lastOrder;
        lastOrder =
                detector.getLastOrder();

        return lastOrder;
    }

    public void disableVision() {

        detector.disable();
    }

    public SamplingOrderDetector.GoldLocation getSamplingOrderSmart() {

        int leftCount = 0;
        int centerCount = 0;
        int rightCound = 0;
        int unknownCount = 0;

        SamplingOrderDetector.GoldLocation finalOrder = SamplingOrderDetector.GoldLocation.UNKNOWN;

        for(int i = 0; i < 20; i++) {
            SamplingOrderDetector.GoldLocation tempOrder = detector.getCurrentOrder();

            if(tempOrder == SamplingOrderDetector.GoldLocation.LEFT) {
                leftCount = leftCount + 1;
            } else if(tempOrder == SamplingOrderDetector.GoldLocation.RIGHT) {
                rightCound = rightCound + 1;
            } else if(tempOrder == SamplingOrderDetector.GoldLocation.CENTER) {
                centerCount = centerCount + 1;
            } else if(tempOrder == SamplingOrderDetector.GoldLocation.UNKNOWN) {
                unknownCount = unknownCount + 1;
            }
        }


        if(leftCount > rightCound && leftCount > centerCount) {
            finalOrder =  SamplingOrderDetector.GoldLocation.LEFT;
        }
        else if(centerCount > rightCound && leftCount < centerCount) {
            finalOrder = SamplingOrderDetector.GoldLocation.CENTER;
        }
        else if(leftCount < rightCound && rightCound > centerCount) {
            finalOrder = SamplingOrderDetector.GoldLocation.RIGHT;
        }
        else {
            finalOrder = SamplingOrderDetector.GoldLocation.UNKNOWN;
        }

        return finalOrder;
    }
}
