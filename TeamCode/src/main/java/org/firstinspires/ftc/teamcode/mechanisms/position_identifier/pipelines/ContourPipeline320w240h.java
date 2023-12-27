package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline320w240h extends OpenCvPipeline{
    // Green                        Y      Cr     Cb    (Do not change Y)
    // use this picture for you own color https://raw.githubusercontent.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/main/7e8azlgi.bmp
    Scalar GREEN = new Scalar(0, 0, 255);
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(220.0, 255.0, 90.0);
    // public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.50, 0.50);
    // public static Scalar scalarUpperYCrCb = new Scalar(150.0, 150.0, 130.0);

    Telemetry telemetry;

    public double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private Mat mat = new Mat();
    private Mat processed = new Mat();
    private Mat output = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);
    private Rect rect = new Rect(600,1,1,1);


    public ContourPipeline320w240h(Telemetry t) {

        telemetry = t;

        this.borderLeftX = 0.25;
        this.borderRightX = 0.25;
        this.borderTopY = 0.25;
        this.borderBottomY = 0.45;

        // Green Range                                      Y      Cr     Cb
        // Scalar initScalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
        // Scalar initScalarUpperYCrCb = new Scalar(150.0, 150.0, 110.0);
        Scalar initScalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
        Scalar initScalarUpperYCrCb = new Scalar(220.0, 255.0, 90.0);
        configureScalarLower(initScalarLowerYCrCb.val[0],initScalarLowerYCrCb.val[1],initScalarLowerYCrCb.val[2]);
        configureScalarUpper(initScalarUpperYCrCb.val[0],initScalarUpperYCrCb.val[1],initScalarUpperYCrCb.val[2]);
    }


    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();

        // Process Image, convert to RGB, then processed to YCrCb,
        //Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);

        // Remove Noise
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

        // GaussianBlur
        Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);

        // Find Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw Contours, red lines that show color areas that match
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));
        telemetry.addLine("Drawing countours");

        // Show the bounding area in which we will search for countours
        Imgproc.rectangle(input, new Rect(40, 26, 200, 80), new Scalar(0, 0, 255), 2); // BLUE

        // Set default maxRect to one pixel. Default will return as Level 3
        maxRect = new Rect(0,0,1,1);

        // Loop Through Contours, find the counter with matching max and min area
        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();

            // Bound Rectangle if Contour is Large Enough
            if (contourArray.length >= 1) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                Rect rect = Imgproc.boundingRect(areaPoints);

                if (
                        (rect.area() > 1100 && rect.area() < 3000)
                                && rect.x > 40 && rect.x < 270
                ){
                    maxRect = rect;
                    Imgproc.rectangle(input, maxRect, new Scalar(255, 255, 255), 1); // GREEN

                    telemetry.addData("maxrectX", maxRect.x);
                    telemetry.addData("maxrectY", maxRect.y);
                    telemetry.addData("maxrectArea", maxRect.area());
                }
                areaPoints.release();
            }
            contour.release();
        }

        // Outline found rectangle in Green
        Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2); // GREEN

        // Show target locations for midpoint

        Imgproc.rectangle(input, new Rect(40, 30, 98, 78), new Scalar(255, 255, 255), 2); // WHITE BOX Level 1
        Imgproc.rectangle(input, new Rect(140, 30, 78, 78), new Scalar(255, 255, 255), 2); // WHITE BOX Level 2


        // Check maxRect for midpoint value to determine which location the element is in
        if( getRectMidpointXY().x > 40 &&  getRectMidpointXY().x < 139 ) {
            Positions.getInstance().setTEPosition(Positions.TEPosition.POSITION_1);

        } else if( getRectMidpointXY().x > 140 &&  getRectMidpointXY().x < 220 ) {
            Positions.getInstance().setTEPosition(Positions.TEPosition.POSITION_2);
        } else {
            Positions.getInstance().setTEPosition(Positions.TEPosition.POSITION_3);
        }

        // Display Data
        Imgproc.putText(input, "Location" + Positions.getInstance().getTEPosition(), new Point(10, 20), 0, 0.35, new Scalar(255, 255, 255), 1);
        Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(10, 10), 0, 0.35, new Scalar(255, 255, 255), 1);

        telemetry.addData("level", Positions.getInstance().getTEPosition());
        telemetry.update();

        return input;
    }

    public int getRectHeight() {
        return maxRect.height;
    }

    public int getRectWidth() {
        return maxRect.width;
    }

    public int getRectX() {
        return maxRect.x;
    }

    public int getRectY() {
        return maxRect.y;
    }

    public double getRectMidpointX() {
        return getRectX() + (getRectWidth() / 2.0);
    }

    public double getRectMidpointY() {
        return getRectY() + (getRectHeight() / 2.0);
    }

    public Point getRectMidpointXY() {
        return new Point(getRectMidpointX(), getRectMidpointY());
    }

    public double getAspectRatio() {
        return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
    }

    public double getRectArea() {
        return maxRect.area();
    }

    public Positions.TEPosition getPosition() {
        return Positions.getInstance().getTEPosition();
    }

}
