package org.firstinspires.ftc.teamcode.Main;

import org.firstinspires.ftc.teamcode.Main.Vision.CameraCropAngle;

/**
 * Created by adityamavalankar on 11/4/18.
 */

public class Constants {

    /**
     * The constants program is meant to make things consistent throughout our programs, without much fiddling with the code.
     * The benefit of having constant names is that one configuration file works with everything!
     *
     * Again, modifying ANYTHING here is getting into the DANGER ZONE! Please talk to someone else before producing any changes
     * to these values. In the time being until that, you may create a new variable.
     */
    public final String LEFT_FRONT_NAME = "leftFront";
    public final String RIGHT_FRONT_NAME = "rightFront";
    public final String LEFT_BACK_NAME = "leftBack";
    public final String RIGHT_BACK_NAME = "rightBack";
    public final String COLLECTOR_NAME = "collector";
    public final String LIFT_SLIDES_NAME = "hanger";
    public final String EXTENSION_NAME = "extension";
    public final String TILT_MOTOR_NAME = "tiltMotor";

    //names for the sensors
    public final String GYRO_NAME = "imu";
    public final String WEBCAM_FRONT_NAME = "webcamFront";
    public final String LEFT_COLOR_NAME = "leftLine";
    public final String RIGHT_COLOR_NAME = "rightLine";
    public final String RIGHT_DISTANCE_NAME = "rightLineDist";
    public final String LEFT_DISTANCE_NAME = "leftLineDist";
    public final String GROUND_DISTANCE_SENSOR_NAME = "groundDistance";
    public final String WALL_ALIGN_FRONT_NAME = "wallAlignFront";

    //names for servos
    public final String LEFT_MINERAL_NAME = "leftMineral";
    public final String RIGHT_MINERAL_NAME = "rightMineral";
    public final String MINERAL_FEEDER_NAME = "mineralFeeder";
    public final String DEPOSITER_ROTATE_NAME = "depositerRotate";
    public final String DEPOSITER_DUMP_NAME = "depositerDump";
    public final String INTAKE_LIFTER_NAME = "intakeLifter";
    public final String MARKER_DEPOSITER_NAME = "markerDepositer";
    public final String RIGHT_LOCK_SERVO_NAME = "rightLockServo";
    public final String LEFT_LOCK_SERVO_NAME = "leftLockServo";
    public final String DUMPER_SERVO_NAME = "dumperServo";

    //Vuforia license key, DO NOT TOUCH!
    public final String VUFORIA_KEY = "AffveYv/////AAAAGQ5VbB9zQUgjlHWrneVac2MnNgfMDlq6EwI3tyURgRK6C" +
            "HargOTFidfzKod6GLQwGD4m9MPLkR+0NfUrnY8+o8FqAKcQbrAsjk8ONdkWYTPZDfoBRgDLNWRuB7LU1MOp9KqAWpXB" +
            "JjvH5JCKF/Hxz+beHfVqdWQ0BVZdgGMXG4yEzLN5AI+4NIkQeLvI7Cwz5pIlksoH+rb/e6+YExoWZbQWhDTiRiemlWjvDM" +
            "1z2a0kteGDz0wTyHz48IkV4M0YsSQIFKwu3YB2a1vkB9FiRfMrBI+CyInjgNoO8V0EEOtRc6Vqsf3XbF3fGXricZUhl7RIl5" +
            "M/IkFOgeAZ4ML+JcrjTqfZb2Yh3JNx1me524cK";
    public final CameraCropAngle CAMERA_AIM_DIRECTION = CameraCropAngle.RIGHT;

    public final double P_TURN_COEFF = 0.06;
    public final double P_WALL_COEFF = 0.08;

    public final double DISTANCE_THRESHOLD = 2;
    public final double TURN_THRESHOLD = 2.5;

    //constant numbers meant for autonomous with encoders
    public final double TICKS_PER_ROTATION = 1120;

    public final double LIFT_GEAR_DIAMETER_IN = 1.5;
    public final double LIFT_GEAR_CIRCUMFERENCE_IN = LIFT_GEAR_DIAMETER_IN * Math.PI;

    public final double LIFT_TICKS_PER_IN = TICKS_PER_ROTATION*LIFT_GEAR_CIRCUMFERENCE_IN;

    public final double GEAR_RATIO = 48/72;
    public final double WHEEL_DIAMETER_CM = 10.16;
    public final double WHEEL_DIAMETER_IN = 4;
    public final double WHEEL_CIRCUMFERENCE_CM = (Math.PI * WHEEL_DIAMETER_CM);
    public final double WHEEL_CIRCUMFERENCE_IN = (Math.PI * WHEEL_DIAMETER_IN);

    public final double TICKS_PER_CM = (TICKS_PER_ROTATION)/(WHEEL_CIRCUMFERENCE_CM);
    public final double TICKS_PER_IN = (TICKS_PER_ROTATION)/(WHEEL_CIRCUMFERENCE_IN);

    public final double ENC_DRIVE_TIME_MULTIPLIER = 2;
    public final int NEVEREST_40_RPM = 160;
    public final int LIFT_MOTOR_LOWER_CONSTANT = -1;

    public final int LIFT_SLIDES_REVERSE_CONSTANT = -1;

    public final double ON_GROUND_DISTANCE_CM = 2;

    public final double ROBOT_WIDTH_IN = 13.5;
    public final double ROBOT_CIRCUMFERENCE = (ROBOT_WIDTH_IN * Math.PI);

    public final double ENCODERS_PER_360 = (ROBOT_CIRCUMFERENCE*TICKS_PER_IN);
    public final double ENCODERS_PER_DEGREE = (ENCODERS_PER_360/360);

    // figure this out at the lab - horizontal position on the tape relative to the lander.
    public final double HORIZONTAL_POS_LANDER = 0;

    public final double LEFT_LOCK_SERVO_CLOSED_POS = 0;
    public final double LEFT_LOCK_SERVO_OPEN_POS = 0;
    public final double RIGHT_LOCK_SERVO_CLOSED_POS = 0;
    public final double RIGHT_LOCK_SERVO_OPEN_POS = 0;

}
