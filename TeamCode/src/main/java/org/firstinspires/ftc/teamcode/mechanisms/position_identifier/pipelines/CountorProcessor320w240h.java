package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.vision.VisionProcessor;
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

public class CountorProcessor320w240h implements VisionProcessor {

    Scalar GREEN = new Scalar(0, 0, 255);
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(220.0, 255.0, 90.0);

    public double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private Mat mat = new Mat();
    private Mat processed = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);

    Telemetry telemetry;

    Selected selection = Selected.NONE;
    public CountorProcessor320w240h(Telemetry t){
        telemetry = t;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Code executed on the first frame dispatched into this VisionProcessor
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Actual computer vision magic will happen here
        CAMERA_WIDTH = frame.width();
        CAMERA_HEIGHT = frame.height();

        // Process Image, convert to RGB, then processed to YCrCb,
        //Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2YCrCb);
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
        Imgproc.drawContours(frame, contours, -1, new Scalar(255, 0, 0));
        telemetry.addLine("Drawing countours");

        // Show the bounding area in which we will search for countours
        Imgproc.rectangle(frame, new Rect(40, 26, 200, 80), new Scalar(0, 0, 255), 2); // BLUE

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
                    Imgproc.rectangle(frame, maxRect, new Scalar(255, 255, 255), 1); // GREEN

                    telemetry.addData("maxrectX", maxRect.x);
                    telemetry.addData("maxrectY", maxRect.y);
                    telemetry.addData("maxrectArea", maxRect.area());
                }
                areaPoints.release();
            }
            contour.release();
        }



        return maxRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Cool feature: This method is used for drawing annotations onto
        // the displayed image, e.g outlining and indicating which objects
        // are being detected on the screen, using a GPU and high quality
        // graphics Canvas which allow for crisp quality shapes.

        Mat input = (Mat) userContext;
        Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2); // GREEN

        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 2);

        canvas.drawRect(makeGraphicsRect(maxRect, scaleBmpPxToCanvasPx), rectPaint);

        // Show target locations for midpoint

        Rect position1 = new Rect(40, 30, 98, 78);
        Rect position2 = new Rect(140, 30, 78, 78);
        Imgproc.rectangle(input, position1, new Scalar(255, 255, 255), 2); // WHITE BOX Position 1
        Imgproc.rectangle(input, position2, new Scalar(255, 255, 255), 2); // WHITE BOX Position 2

        Paint rectWhitePaint = new Paint();
        rectPaint.setColor(Color.WHITE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 2);

        canvas.drawRect(makeGraphicsRect(position1, scaleBmpPxToCanvasPx), rectWhitePaint);
        canvas.drawRect(makeGraphicsRect(position2, scaleBmpPxToCanvasPx), rectWhitePaint);


        // Check maxRect for midpoint value to determine which location the element is in
        if( getRectMidpointXY().x > 40 &&  getRectMidpointXY().x < 139 ) {
            //Positions.getInstance().setTEPosition(Positions.TEPosition.POSITION_1);
            selection = Selected.LEFT;


        } else if( getRectMidpointXY().x > 140 &&  getRectMidpointXY().x < 220 ) {
            //Positions.getInstance().setTEPosition(Positions.TEPosition.POSITION_2);
            selection = Selected.MIDDLE;
        } else {
            //Positions.getInstance().setTEPosition(Positions.TEPosition.POSITION_3);
            selection = Selected.RIGHT;
        }

        // Display Data
        Imgproc.putText(input, "Location" + selection, new Point(10, 20), 0, 0.35, new Scalar(255, 255, 255), 1);
        Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(10, 10), 0, 0.35, new Scalar(255, 255, 255), 1);


        telemetry.update();
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
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

    //public Positions.TEPosition getPosition() {
        //return Positions.getInstance().getTEPosition();
    //}

    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}


