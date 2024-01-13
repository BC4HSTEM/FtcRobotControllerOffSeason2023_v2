package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.globals.Side;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.TSEProcessorLeft;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.TSEProcessorRight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class PositionIdentifierSubsystem extends SubsystemBase {


    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;

    private Telemetry telemetry;

    private int width = 320;
    private int height = 240;

    private Positions.TEPosition localPosition = Positions.TEPosition.NONE;

    private boolean positionSet = false;


    public PositionIdentifierSubsystem(VisionPortal.Builder vpb, Telemetry t){

        visionPortalBuilder = vpb;
        telemetry = t;
        VisionProcessor tseProcessor = null;

        if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
            tseProcessor = new TSEProcessorLeft(this, telemetry);
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
            tseProcessor = new TSEProcessorRight(this, telemetry);
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
            tseProcessor = new TSEProcessorRight(this, telemetry);
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
            tseProcessor = new TSEProcessorLeft(this, telemetry);
        }
             // Other choices are: RC phone camera and "switchable camera name".

// Add the AprilTag Processor to the VisionPortal Builder.
        visionPortalBuilder.addProcessor(tseProcessor);       // An added Processor is enabled by default.

// Optional: set other custom features of the VisionPortal (4 are shown here).
        visionPortalBuilder.setCameraResolution(new Size(width, height));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);  // MJPEG format uses less bandwidth than the default YUY2.
        visionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).
        visionPortalBuilder.setAutoStopLiveView(true);

    }


    public void setPosition(Positions.TEPosition position){

        localPosition = position;
        positionSet = true;


    }


    public Positions.TEPosition getLocalPosition(){

        return localPosition;
    }


    public void stopStreaming(){

        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
            visionPortal.close();
        }
    }

    public void closeStream(){
        visionPortal.close();
    }

    public void detectTEPosition(){
        visionPortal = visionPortalBuilder.build();
    }

    public boolean isPositionSet(){
        return positionSet;
    }

    public Positions.TEPosition getPosition(){
        return localPosition;
    }
}
