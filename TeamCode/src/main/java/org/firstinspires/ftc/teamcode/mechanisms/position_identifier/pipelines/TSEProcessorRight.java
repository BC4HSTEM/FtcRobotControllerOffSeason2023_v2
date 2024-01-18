package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class TSEProcessorRight implements VisionProcessor {

    public static Rect rectLeft = new Rect(15, 160, 40, 40);
    public static Rect rectMiddle = new Rect(125, 130, 40, 40);

    public static Rect rectRight = new Rect(250, 160, 40, 40);

    Telemetry telemetry;
    PositionIdentifierSubsystem positionIdentifierSubsystem;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    Positions.TEPosition selection = Positions.TEPosition.NONE;
    //Selected selection = Selected.NONE;

    public TSEProcessorRight(PositionIdentifierSubsystem subsystem, Telemetry t){
        super();

        positionIdentifierSubsystem = subsystem;
        telemetry = t;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        //telemetry.addData("satRectLeft", satRectLeft);
        //telemetry.addData("satRectMiddle", satRectMiddle);
        //telemetry.addData("satRectRight", satRectRight);
        //telemetry.update();

        if(satRectLeft + satRectMiddle + satRectRight >= 300){
            return Positions.TEPosition.NONE;
        }
        else if(satRectLeft < 50 && satRectMiddle < 50){
            return Positions.TEPosition.POSITION_RIGHT;
        }
        else if (satRectRight < 50 && satRectMiddle < 50) {
            return Positions.TEPosition.POSITION_LEFT;
        }
        else if (satRectRight < 50 && satRectLeft < 50) {
            return Positions.TEPosition.POSITION_MIDDLE;
        }
        else {
            return Positions.TEPosition.NONE;
        }

    }

    protected double getAvgSaturation(Mat input, Rect rect){
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint();
        nonSelectedPaint.setColor(Color.GREEN);
        nonSelectedPaint.setStyle(Paint.Style.STROKE);
        nonSelectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        //selection = (Selected) userContext;
        selection = (Positions.TEPosition) userContext;
        telemetry.addData("I'm the selected PR", selection);
        //Positions.getInstance().setTEPosition(selection);
        positionIdentifierSubsystem.setPosition(selection);
        telemetry.addData("I'm the selected PR from subsystem", positionIdentifierSubsystem.getPosition());

        /*switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;

            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;

        }*/

        switch (selection) {
            case POSITION_LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case POSITION_MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;

            case POSITION_RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;

        }


    }

    /*public Positions.TEPosition getSelection(){
        return selection;
    }*/


    /*public Selected getSelection()
    {

        return selection;
    }*/



    /*public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }*/
}
