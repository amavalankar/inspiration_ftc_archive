package org.firstinspires.ftc.teamcode.Vision;

/**
 * Created by adityamavalankar on 11/12/18.
 */

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;

import org.firstinspires.ftc.teamcode.Constants;

public class SamplingDetector {

    public SamplingOrderDetector
    detector= new SamplingOrderDetector();
    public Constants constants = new Constants();
    HardwareMap hwmap;

    public void initVision(HardwareMap ahwmap) {

        initVision(ahwmap, constants.CAMERA_AIM_DIRECTION);

    }

    public void initVision(HardwareMap ahwmap, CameraCropAngle cropAngle) {
        hwmap = ahwmap;

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


        detector.init(hwmap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera

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
