
/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.hardwaremaps.ExternalCameraHardwareMap;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.TimeUnit;


/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * Android Camera2 API
 */
//@Disabled
@TeleOp(name="Teleop: Camera Exposure Test", group="Teleop")
public class CameraExposureTest extends LinearOpMode

{
    int width = 320;
    int height = 240;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // TSEProcessorRight
    public static Rect rectLeft = new Rect(65, 160, 40, 40);
    public static Rect rectMiddle = new Rect(165, 150, 40, 40);
    public static Rect rectRight = new Rect(280, 160, 40, 40);

    // Exposure and Gain Control Test
    ExposureControl myExposureControl;
    GainControl myGainControl;
    FocusControl myFocusControl;
    WhiteBalanceControl myWhiteBalanceControl;

    @Override
    public void runOpMode()
    {
        ExternalCameraHardwareMap robot = new ExternalCameraHardwareMap();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //OpenCvShippingElementDetector detector = new OpenCvShippingElementDetector( width, height, telemetry);
        SamplePipeline detector = new SamplePipeline();
        robot.init(hardwareMap);
        robot.webCam.setPipeline(detector);



        robot.webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */

                robot.webCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

                // Setting manual values here in place of auto
                robot.webCam.getFocusControl().setMode(FocusControl.Mode.Infinity);
                robot.webCam.getExposureControl().setAePriority(false);
                robot.webCam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                robot.webCam.getExposureControl().setExposure(33, TimeUnit.MILLISECONDS);
                robot.webCam.getGainControl().setGain(182);
                robot.webCam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                robot.webCam.getWhiteBalanceControl().setWhiteBalanceTemperature(2200);




            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */

            }
        });


        telemetry.addLine("Waiting for start");
        dashboard.startCameraStream(robot.webCam, 20);
        telemetry.addLine("Streaming started");


        myExposureControl = robot.webCam.getExposureControl();
        myWhiteBalanceControl = robot.webCam.getWhiteBalanceControl();
        myGainControl = robot.webCam.getGainControl();
        myFocusControl = robot.webCam.getFocusControl();

        /*
        myExposureControl.setAePriority(false);
        myExposureControl.setExposure(40, TimeUnit.MILLISECONDS);
        myGainControl.setGain(205);
        myWhiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        myWhiteBalanceControl.setWhiteBalanceTemperature(2000);

         */

        while(!isStarted()){
            telemetry.addData("Gain Control", myGainControl.getGain());
            telemetry.addData("Aperture Priority (AE)?", myExposureControl.getAePriority());
            telemetry.addData("Focus Length (0-250)", myFocusControl.getFocusLength());
            telemetry.addData("Get Exposure", myExposureControl.getExposure(TimeUnit.MILLISECONDS));
            telemetry.addData("White Balance Value", myWhiteBalanceControl.getWhiteBalanceTemperature());
            telemetry.addData("Exposure Control Mode", myExposureControl.getMode());
            telemetry.addData("White Balance Mode", myWhiteBalanceControl.getMode());
            telemetry.update();
        }

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */

            if(gamepad1.a){
                myExposureControl.setAePriority(false);
                myExposureControl.setMode(ExposureControl.Mode.Manual);
            } else if(gamepad1.b){
                myExposureControl.setAePriority(true);
                myExposureControl.setMode(ExposureControl.Mode.AperturePriority);
            } else if(gamepad1.x){
                myFocusControl.setFocusLength(0);
            }

            telemetry.addData("Exposure Control Mode", myExposureControl.getMode());
            telemetry.addData("Gain Control", myGainControl.getGain());
            telemetry.addData("Aperture Priority (AE)?", myExposureControl.getAePriority());
            telemetry.addData("Focus Length (0-250)", myFocusControl.getFocusLength());
            telemetry.addData("Focus Length Min", myFocusControl.getMinFocusLength());
            telemetry.addData("Focus Length Max", myFocusControl.getMaxFocusLength());
            telemetry.addData("Get Exposure", myExposureControl.getExposure(TimeUnit.MILLISECONDS));

            telemetry.addData("Frame Count", robot.webCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", robot.webCam.getFps()));
            telemetry.addData("Total frame time ms", robot.webCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", robot.webCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", robot.webCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", robot.webCam.getCurrentPipelineMaxFps());
            telemetry.update();

            

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(10);
        }

    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */

            Mat resizeimage = new Mat();
            Size scaleSize = new Size(width,height);
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2RGB);
            Imgproc.resize(input, resizeimage, scaleSize , 0, 0, Imgproc.INTER_AREA);

            Imgproc.putText(input, "Team prop locator", new Point(10,20), 0, 0.35, new Scalar(255,255,255));

            // Check TSEPositionRight
            Imgproc.rectangle(input, rectLeft, new Scalar(255, 255, 255), 2); // WHITE BOX Level 1
            Imgproc.rectangle(input, rectMiddle, new Scalar(255, 255, 255), 2); // WHITE BOX Level 2
            Imgproc.rectangle(input, rectRight, new Scalar(255, 255, 255), 2); // WHITE BOX Level 2



            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }
    }



}
