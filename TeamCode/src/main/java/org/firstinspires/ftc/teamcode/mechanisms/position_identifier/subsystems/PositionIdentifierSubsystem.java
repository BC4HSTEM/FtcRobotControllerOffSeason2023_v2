package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class PositionIdentifierSubsystem extends SubsystemBase {

    private OpenCvWebcam webCam;
    private OpenCvPipeline pipeline;

    private int msPermTo = 2500;

    private int width = 320;
    private int height = 240;

    private static final int MAX_FRAMES_PER_SECOND = 10;
    private static final String ID_NAME = "cameraMonitorViewId";
    private static final String ID_DEF_TYPE = "id";

    //private OpenCvShippingElementDetector.TSELocation location;
    private int currrentPosition = 0;

    public PositionIdentifierSubsystem(OpenCvWebcam wc){

        webCam = wc;

    }

    public PositionIdentifierSubsystem(OpenCvWebcam wc, OpenCvPipeline pipeline){

        webCam = wc;
        setPipeline(pipeline);
    }

    public PositionIdentifierSubsystem(OpenCvWebcam wc, int msPermTo){

        webCam = wc;
        setMSPermTO(msPermTo);
    }

    public PositionIdentifierSubsystem(OpenCvWebcam wc, int msPermTo, int width, int height){

        webCam = wc;
        //webCam.getFocusControl();
        setMSPermTO(msPermTo);
        setWidth(width);
        setHeight(height);
    }

    public void setMSPermTO(int msPermTO){
        msPermTo = msPermTO;
    }

    public void setWidth(int w){
        width = w;
    }

    public void setHeight(int h){
        height = h;
    }

    public void setPipeline(OpenCvPipeline pipeline){
        this.pipeline = pipeline;
        webCam.setPipeline(pipeline);
    }

    public OpenCvPipeline getPipeline(){
        return pipeline;
    }

    public void StreamToDashboard(FtcDashboard dashboard){
        dashboard.startCameraStream(webCam, MAX_FRAMES_PER_SECOND);
    }

    public void openCameraDeviceAsync(){
        webCam.setMillisecondsPermissionTimeout(5000);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
                webCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */

            }
        });
    }

    public void setPosition(Positions.TEPosition position){
        Positions.getInstance().setTEPosition(position);

    }


    public Positions.TEPosition getPosition(){
        return Positions.getInstance().getTEPosition();
    }


    public void stopStreaming(){
        webCam.stopStreaming();
    }

    public void closeStream(){
        webCam.closeCameraDevice();
    }
}
