package com.disnodeteam.dogecv.detectors.roverrukus;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 9/10/2018.
 */

public class SamplingOrderDetector extends DogeCVDetector {

    // Enum to describe gold location
    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }



    // Which area scoring method to use
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

    //Create the scorers used for the detector
    public RatioScorer ratioScorer             = new RatioScorer(1.0,5);
    public MaxAreaScorer maxAreaScorer         = new MaxAreaScorer(0.01);
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05);

    //Create the filters used
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW,100);
    public DogeCVColorFilter whiteFilter  = new HSVRangeFilter(new Scalar(0,0,200), new Scalar(50,40,255));


    // Results for the detector
    private GoldLocation currentOrder = GoldLocation.UNKNOWN;
    private GoldLocation lastOrder    = GoldLocation.UNKNOWN;
    private boolean      isFound      = false;
    public boolean positionCamLeft = false;
    public boolean positionCamRight = false;

    // Create the mats used
    private Mat origin = new Mat();
    private Mat workingMat;
    private Mat displayMat;
    private Mat yellowMask  = new Mat();

    private Mat hiarchy     = new Mat();
    public Rect roi = new Rect(0, 300, 640, 180);


    public SamplingOrderDetector() {
        super();
        this.detectorName = "Sampling Order Detector";
    }

    @Override
    public Mat process(Mat input) {

        // Copy input mat to working/display mats
        input.copyTo(origin);
        input.release();

        workingMat = new Mat(origin.clone(), roi);
        displayMat = new Mat(origin.clone(), roi);

        // Generate Masks
        yellowFilter.process(workingMat.clone(), yellowMask);



        // Blur and find the countours in the masks
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursWhite = new ArrayList<>();

        Imgproc.blur(yellowMask,yellowMask,new Size(2,2));

        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);


        // Prepare to find best yellow (gold) results
        Rect   chosenYellowRect  = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursYellow){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double diffrenceScore = calculateScore(points);

            if(diffrenceScore < chosenYellowScore && diffrenceScore < maxDifference){
                chosenYellowScore = diffrenceScore;
                chosenYellowRect = rect;
            }

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if( area > 500){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }
        }



        //Draw found gold element
        if(chosenYellowRect != null){
            Imgproc.rectangle(displayMat,
                    new Point(chosenYellowRect.x, chosenYellowRect.y),
                    new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(displayMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenYellowScore, (double)chosenYellowRect.x),
                    new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);

        }


        // If enough elements are found, compute gold position
        if(chosenYellowRect != null){
            int leftCount = 0;

            if(chosenYellowRect.x < 320) {

                currentOrder = SamplingOrderDetector.GoldLocation.LEFT;

            } else if(chosenYellowRect.x > 320) {

                currentOrder = SamplingOrderDetector.GoldLocation.CENTER;

            } else {

                currentOrder = SamplingOrderDetector.GoldLocation.RIGHT;

            }


            lastOrder = currentOrder;

        }


        //Display Debug Information
        Imgproc.putText(displayMat,"Gold Position: " + currentOrder.toString(),new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);
        Imgproc.putText(displayMat,"Last Track: " + lastOrder.toString(),new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);

        return displayMat;
    }

    @Override
    public void useDefaults() {
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
        addScorer(ratioScorer);
    }

    public void setCroppedAngle(boolean useRight) {
        if (useRight) {
            positionCamRight = true;
            positionCamLeft = false;
        }

        else if (useRight == false) {
            positionCamRight = false;
            positionCamLeft = true;
        }

        else {
            positionCamLeft = false;
            positionCamRight = false;
        }
    }

    /**
     * Is both elements found?
     * @return if the elements are found
     */
    public boolean isFound() {
        return isFound;
    }

    /**
     * Returns the current gold pos
     * @return current gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldLocation getCurrentOrder() {
        return currentOrder;
    }

    /**
     * Returns the last known gold pos
     * @return last known gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldLocation getLastOrder() {
        return lastOrder;
    }
}
