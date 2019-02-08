package org.firstinspires.ftc.teamcode.OpModes;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constant.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import com.qualcomm.robotcore.util.Range;


/**
 * Created by adityamavalankar on 11/19/18.
 */

public abstract class ExtendedLinearOpMode extends LinearOpMode {

    /**
     * This is a modified version of the {LinearOpMode} class, with all of the functions
     * meant for autonomous, without having extra work to make them, and the autonomous LinearOpMode
     * to both work
     */

    public Robot robot = new Robot();
    public Constants constants = new Constants();
    private ElapsedTime runtime = new ElapsedTime();
    public Orientation angles;


    /**
     * @param leftTarget
     * @param rightTarget
     */
    public void setTargetPosition(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + rightTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);

    }

    public void setTargetPositionStrafe(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + leftTarget);
        robot.rightBack.setTargetPosition(robot.rightFront.getCurrentPosition() + leftTarget);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + rightTarget);
        robot.rightFront.setTargetPosition(robot.rightBack.getCurrentPosition() + rightTarget);
    }

    public void setTargetPositionReset(int leftTarget, int rightTarget) {
        robot.leftFront.setTargetPosition(leftTarget);
        robot.rightFront.setTargetPosition(rightTarget);
        robot.leftBack.setTargetPosition(leftTarget);
        robot.rightBack.setTargetPosition(rightTarget);
    }

    protected void resetEncoderAngle() {
        robot.encoderTurnAngle = 0;
    }

    protected void setEncoderAngle(int angle) {
        robot.encoderTurnAngle = angle;
    }

    public void setPower(double power) {
        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

    }

    public void setPowerStrafe(double power1, double power2) {
        robot.leftFront.setPower(power2);
        robot.leftBack.setPower(power1);
        robot.rightFront.setPower(power1);
        robot.rightBack.setPower(power2);

    }

    public void setPower(double left_power, double right_power) {
        robot.leftFront.setPower(left_power);
        robot.leftBack.setPower(left_power);
        robot.rightFront.setPower(right_power);
        robot.rightBack.setPower(right_power);

    }

    public void doEncoderTurn(double speed, int angle) {

        int tgAngle = Math.abs(angle);
        double distance;
        double leftDistance;
        double rightDistance;

        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distance = ((constants.ENCODERS_PER_DEGREE * tgAngle) / constants.TICKS_PER_IN);


        telemetry.addLine("Target Positions Calculated");
        telemetry.update();


        if (angle > 0) {
            rightDistance = -distance;
            leftDistance = distance;


            encoderDriveIN(leftDistance, rightDistance, speed, 5.5);
        } else if (angle < 0) {
            leftDistance = -distance;
            rightDistance = distance;


            encoderDriveIN(leftDistance, rightDistance, speed, 5.5);
        }

        else {

            encoderDriveIN(0, 0, speed, 5.5);

        }

    }

    public void encoderTurn(double speed, int angle) {

        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (robot.encoderTurnAngle == angle) {
            idle();
            telemetry.addLine("Turn Complete!");
            telemetry.update();
        } else {
            int targetAngle = (angle - robot.encoderTurnAngle);

            sleep(10);
            setEncoderAngle(angle);


            doEncoderTurn(speed, targetAngle);

            sleep(10);
            telemetry.addLine("Turn Complete!");
            telemetry.update();

        }




        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        sleep(50);

    }

    public void encoderStrafe(double units, double speed) {

        int left_distanceEnc = (int) (constants.TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (constants.TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            // setPowerStrafe(Math.abs(speed), Math.abs(speed));
            setPower(Math.abs(speed));

            while (opModeIsActive() && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", units);
                telemetry.addData("Target Distance Right (in)", units);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }

            setPower(0);

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }

    public void encoderStrafeOffset(double units, double speed, double offset1, double offset2, double timeoutS) {

        int left_distanceEnc = (int) (constants.TICKS_PER_IN * -units);
        int right_distanceEnc = (int) (constants.TICKS_PER_IN * units);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setTargetPositionStrafe(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            // setPowerStrafe(Math.abs(speed), Math.abs(speed));
            setPowerStrafe(Math.abs(speed-offset1), Math.abs(speed-offset2));

            runtime.reset();

            while (opModeIsActive() && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && (runtime.seconds() < timeoutS)) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", units);
                telemetry.addData("Target Distance Right (in)", units);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }

            setPower(0);

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }


    /**
     * We first convert the inputted distance (in cm) to encoder ticks with the constants.TICKS_PER_CM constant, while
     * setting motors to the runMode RUN_TO_POSITION.
     * Then, we see the expected time the motors should run at using the { timeSec() } function
     * Then, we set the target position, set motors to move at desired speed, until we hit the correct position
     * Then, we set speed to zero
     *
     * @param left_cm
     * @param speed
     */
    public void encoderDriveCM(double left_cm, double right_cm, double speed, double timeoutS) {


        int left_distanceEnc = (int) (constants.TICKS_PER_CM * left_cm);
        int right_distanceEnc = (int) (constants.TICKS_PER_CM * right_cm);

        setTargetPosition(left_distanceEnc, right_distanceEnc);
        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(Math.abs(speed));
        runtime.reset();

        while (opModeIsActive() && robot.leftFront.isBusy() && (runtime.seconds() < timeoutS) && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {
            telemetry.addLine("Robot in Encoder Drive");
            telemetry.addData("Target Distance Left (cm)", left_cm);
            telemetry.addData("Target Distance Right (cm)", right_cm);
            telemetry.update();
        }

        setPower(0);

        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void tiltMarker(double seconds, double power) {

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {

            robot.tiltMotor.setPower(power);

        }
        robot.tiltMotor.setPower(0);

    }

    public void encoderDriveINNew(double left_in, double right_in, double speed, double timeoutS) {

        int leftTarget = (int) (Math.ceil(89 * left_in * 2/3));
        int rightTarget = (int) (Math.ceil(89 * right_in * 2/3));

        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTargetPosition(leftTarget, rightTarget);
        telemetry.addData("posleft", leftTarget);

        setPower(Math.abs(speed));
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {
            telemetry.addLine("Robot in Encoder Drive");
            telemetry.addData("Target Distance Left (in)", left_in);
            telemetry.addData("Target Distance Right (in)", right_in);
            telemetry.update();
            //just one more test...
        }


        setPower(0);

        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);


    }


    public void encoderDriveMotor(DcMotor inputMotor, double in, double speed, double timeoutS) {

        DcMotor motorToRun = inputMotor;

        int targetDist = (int)(constants.TICKS_PER_IN*in);
        motorToRun.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorToRun.setTargetPosition(targetDist + motorToRun.getCurrentPosition());
        sleep(250);

        motorToRun.setPower(Math.abs(speed));
        runtime.reset();

        while(opModeIsActive() && (runtime.seconds() < timeoutS) && motorToRun.isBusy()) {
            telemetry.addLine("Motor in Encoder Drive");
            telemetry.addData("Target Distance (in)", in);
            telemetry.update();
            idle();
            //just one more test...
            motorToRun.setPower(Math.abs(speed));

        }

        motorToRun.setPower(0);

        motorToRun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }

    public void moveActuator(double in) {
        //robot.liftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        robot.liftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int targetPosition = (robot.liftSlides.getCurrentPosition() + (int)(constants.LIFT_TICKS_PER_IN*in * 2/3));
        robot.liftSlides.setTargetPosition(targetPosition);
        sleep(100);

        robot.liftSlides.setPower(Math.abs(1));

        while(opModeIsActive() && robot.liftSlides.isBusy()) {

            telemetry.addData("Current Position", robot.liftSlides.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();

        }
        telemetry.addLine("Target position reached");
        robot.liftSlides.setPower(0);
        robot.liftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(250);

    }

    public void moveActuator(double in, double timeoutS) {
        //robot.liftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        robot.liftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int targetPosition = (robot.liftSlides.getCurrentPosition() + (int)(constants.LIFT_TICKS_PER_IN*in * 2/3));
        robot.liftSlides.setTargetPosition(targetPosition);
        sleep(100);

        robot.liftSlides.setPower(Math.abs(1));

        runtime.reset();
        while(opModeIsActive() && robot.liftSlides.isBusy() && (runtime.seconds() < timeoutS)) {

            telemetry.addData("Current Position", robot.liftSlides.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Time", runtime.seconds());
            telemetry.addData("Target Time", runtime.seconds());
            telemetry.update();

        }

        if (runtime.seconds() > timeoutS) {
            telemetry.addData("Actuator Timed Out!", runtime.seconds());
            telemetry.update();
        }


        telemetry.addLine("Target position reached");
        robot.liftSlides.setPower(0);
        robot.liftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(250);

    }

    public void setMotorRunMode(DcMotor.RunMode runmode) {
        robot.leftFront.setMode(runmode);
        robot.rightFront.setMode(runmode);
        robot.leftBack.setMode(runmode);
        robot.rightBack.setMode(runmode);
    }

    /**
     * The expected drive time is calculated with some math. We multiply speed by RPM of motor, and multiply that
     * by wheel circumference. Then, we divide the distance to be traveled by the total speed per minute.
     * Then, we will convert from minutes to seconds
     * @param cm
     * @param speed
     * @return The total distance that movement should take, multiplied by 1.5 for padding
     */

    public double calculateDriveTimeCM(double cm, double speed) {

        if(cm == 0) {
            return 0;
        }

        else {
            double abs_speed_unc = Math.abs(constants.NEVEREST_40_RPM);
            double abs_distCM = Math.abs(cm);

            double circ = constants.WHEEL_CIRCUMFERENCE_CM;
            double dist_perMin = (abs_speed_unc * circ);
            double timeMin = (dist_perMin / abs_distCM);
            double timeSec = (timeMin * 60);

            return Math.abs(timeSec * constants.ENC_DRIVE_TIME_MULTIPLIER);
        }
    }

    public double calculateDriveTimeIN(double in, double speed) {

        if(in == 0) {
            return 0;
        }

        else {
            double abs_speed_unc = Math.abs(constants.NEVEREST_40_RPM);
            double abs_distIN = Math.abs(in);

            double circ = constants.WHEEL_CIRCUMFERENCE_IN;
            double dist_perMin = (abs_speed_unc * circ);
            double timeMin = (dist_perMin / abs_distIN);
            double timeSec = (timeMin * 60);

            return Math.abs(timeSec * constants.ENC_DRIVE_TIME_MULTIPLIER);
        }
    }

    public void wallAlign (DistanceSensor inputSensor, double speed, double distance) {
        runtime.reset();
        double kP = 0.4;

        double error = (inputSensor.getDistance(DistanceUnit.INCH) - distance);

        while(opModeIsActive() && (Math.abs(error) > 1.7) && (runtime.seconds() < 10)); {
            error = ((inputSensor.getDistance(DistanceUnit.INCH) - distance));
            if(error > 0) {
                speed = Range.clip(error * kP, -1 , 1);
            }
        }

    }

    public void wallAlign(double speed, double distance) {
        runtime.reset();

        while(opModeIsActive() && !onTargetDistance(speed, distance, constants.P_WALL_COEFF) && (runtime.seconds() < 3)){
            telemetry.update();
            idle();
            sleep(200);
        }


        setPower(0);
        telemetry.addLine("Wall Align complete");
        telemetry.update();

    }


    boolean onTargetDistance(double speed, double distance, double PCoeff){
        double errorDistance;
        double steer;
        boolean onTarget = false;
        double finalSpeed;

        //determine turm power based on error
        errorDistance = getErrorDistance(distance);

        if (Math.abs(errorDistance) <= constants.DISTANCE_THRESHOLD){

            steer = 0.0;
            finalSpeed = 0.0;
            onTarget = true;
        }
        else{

            steer = getSteerError(errorDistance, PCoeff);
            finalSpeed = speed * steer;
        }

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(finalSpeed);

        telemetry.addData("Target distance","%5.2f",distance);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", errorDistance, steer);
        telemetry.addData("speed", "%5.2f/%5.2f", finalSpeed, finalSpeed);

        return onTarget;
    }
    public double getErrorDistance(double targetDistance){

        double robotError;

        robotError = robot.wallAlignFront.getDistance(DistanceUnit.INCH) - targetDistance;

        telemetry.addData("Robot Error","%5.2f",robotError);
        telemetry.update();

        return robotError;

    }

    public double getSteerError(double error , double PCoeff){
        return Range.clip(error * PCoeff, -1 , 1);
    }



    public void doSampling(SamplingOrderDetector.GoldLocation samplingOrder) {


        if(samplingOrder == SamplingOrderDetector.GoldLocation.LEFT) {
            //encoderDriveIN(-4.5, 4.5, 0.6, 2);
            encoderTurn(0.2, 50);
            sleep(150);
            encoderDriveIN(30, 30, 0.6, 3);
            sleep(150);
        }
        else if (samplingOrder == SamplingOrderDetector.GoldLocation.CENTER || samplingOrder == SamplingOrderDetector.GoldLocation.UNKNOWN) {
            encoderDriveIN(30, 30, 0.6, 3);
            sleep(150);
        }
        else if (samplingOrder == SamplingOrderDetector.GoldLocation.RIGHT) {
            //encoderDriveIN(-4.5, 4.5, 0.6, 2);
            encoderTurn(0.2, -50);
            sleep(150);
            encoderDriveIN(30, 30, 0.6, 3);
            sleep(150);
        }

        sleep(500);
    }

    /*
    public void doSamplingGyro() {
        SamplingOrderDetector.GoldLocation samplingOrder = robot.getSamplingOrderSmart();

        sleep(500);

        if(samplingOrder == SamplingOrderDetector.GoldLocation.UNKNOWN) {
            samplingOrder = robot.getSamplingOrderSmart();
        }

        if(samplingOrder == SamplingOrderDetector.GoldLocation.LEFT) {
            gyroTurn(0.6, 60, 2);
            sleep(150);
            encoderDriveIN(30, 30, 0.6, 3);
            sleep(150);
        }
        else if (samplingOrder == SamplingOrderDetector.GoldLocation.CENTER) {
            encoderDriveIN(30, 30, 0.6, 3);
            sleep(150);
        }
        else if (samplingOrder == SamplingOrderDetector.GoldLocation.RIGHT) {
            gyroTurn(0.6, -60, 2);
            sleep(150);
            encoderDriveIN(30, 30, 0.6, 3);
            sleep(150);
        }

        sleep(500);
    }
    */

    public void deHang() {
        robot.liftSlides.setPower(1 * constants.LIFT_MOTOR_LOWER_CONSTANT);

        while(!onGround() && opModeIsActive()) {
            idle();
        }
        sleep(250);
        robot.liftSlides.setPower(0);


    }

    public boolean onGround() {
        boolean onGround = false;

        if (robot.groundDistance.getDistance(DistanceUnit.CM) > constants.ON_GROUND_DISTANCE_CM) {
            onGround = false;
        }
        else {
            onGround = true;
        }
        return onGround;
    }

    public void alignTape() {

        // Aligns with tape
        while(!(robot.leftLine.blue() > robot.leftLine.red() + robot.leftLine.green()) && opModeIsActive()) {
            // robot.leftFront.setPower(0.2);
            telemetry.addLine("No");
            telemetry.update();
        }
        while(!(robot.rightLine.blue() > robot.rightLine.red() + robot.rightLine.green()) && opModeIsActive()) {
            // robot.rightFront.setPower(0.2);
            telemetry.addLine("No2");
            telemetry.update();
        }

        if(robot.wallAlignFront.getDistance(DistanceUnit.CM) > constants.HORIZONTAL_POS_LANDER) {

                // Strafe

        }

    }

    public void resetGyro() {
        robot.imu.initialize(robot.parameters);
        robot.composeTelemetry();
    }

    public void gyroTurn(double speed, double angle, double seconds){

        telemetry.addLine("starting gyro turn");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        double begintime= runtime.seconds();
        while(opModeIsActive() && !onTargetAngle(speed, angle, constants.P_TURN_COEFF) && (runtime.seconds() - begintime) < seconds){
            telemetry.update();
            idle();
            telemetry.addData("-->","inside while loop :-(");
            telemetry.update();
        }
        setPower(0);
        telemetry.addLine("done with gyro turn");
        telemetry.update();
    }
    boolean onTargetAngle(double speed, double angle, double PCoeff){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        //determine turm power based on error
        error = getError(angle);

        if (Math.abs(error) <= constants.TURN_THRESHOLD){

            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {

            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            //leftSpeed = -5;
        }

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double weightConstant = 1;//this constant will depend on the robot. you need to test experimentally to see which is best

        setPower(weightConstant*leftSpeed, weightConstant*rightSpeed);

        telemetry.addData("Target angle","%5.2f",angle);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("speed", "%5.2f/%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle){

        double robotError;

        robotError = targetAngle - angles.firstAngle;

        while(opModeIsActive() && robotError > 180) robotError -= 360;

        while(opModeIsActive() && robotError <= -180) robotError += 360;

        telemetry.addData("Robot Error","%5.2f",robotError);
        telemetry.update();

        return robotError;

    }
    public double getSteer(double error , double PCoeff){
        if((error*PCoeff) > 0) {
            return Range.clip(error * PCoeff, 0.2, 1);
        }
        else if(error*PCoeff< 0){
            return Range.clip(error * PCoeff, -1, -0.2);
        }
        else{
            return 0;
        }


    }

    public void leftSample() {

        encoderStrafeOffset(-10, 1, 0, 0.6, 2);
        encoderDriveIN(-18, -18, 0.25, 5);
        encoderDriveIN(18, 18, 0.25, 5);

    }

    public void leftSampleNoBack() {

        encoderStrafeOffset(-10, 1, 0, 0.6, 2);
        encoderDriveIN(-18, -18, 0.25, 5);

    }

    public void centerSample() {

        encoderStrafeOffset(16.5, 1, 0, 0.6, 3);
        encoderDriveIN(-18, -18, 0.25, 5);
        encoderDriveIN(18, 18, 0.25, 5);

    }

    public void centerSampleNoBack() {

        encoderStrafeOffset(16.5, 1, 0, 0.6, 3);
        encoderDriveIN(-18, -18, 0.25, 5);

    }

    public void rightSample() {

        encoderStrafeOffset(41.5, 1, 0, 0.6, 4);
        encoderDriveIN(-18, -18, 0.25, 5);
        encoderDriveIN(18, 18, 0.25, 5);

    }

    public void rightSampleNoBack() {

        encoderStrafeOffset(41.5, 1, 0, 0.6, 4);
        encoderDriveIN(-18, -18, 0.25, 5);

    }

    public void encoderDriveIN(double left_in, double right_in, double speed, double timeoutS) {

        int left_distanceEnc = (int) (constants.TICKS_PER_IN * left_in);
        int right_distanceEnc = (int) (constants.TICKS_PER_IN * right_in);

        // Ensure that the opmode is still active

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);

            setTargetPosition(left_distanceEnc, right_distanceEnc);
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setPower(Math.abs(speed));

            while ((opModeIsActive() && runtime.seconds() < timeoutS) && robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                telemetry.addLine("Robot in Encoder Drive");
                telemetry.addData("Target Distance Left (in)", left_in);
                telemetry.addData("Target Distance Right (in)", right_in);
                telemetry.addData("TickLeft", left_distanceEnc);
                telemetry.addData("TickRight", right_distanceEnc);
                telemetry.update();
                //just one more test...
            }

            setPower(0);

            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }

    }

}