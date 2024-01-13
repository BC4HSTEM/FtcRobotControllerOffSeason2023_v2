package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.vision.VisionPortal;

public class PositionIdentifierSubsystem extends SubsystemBase {


    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;

    private Positions.TEPosition localPosition = Positions.TEPosition.NONE;


    public PositionIdentifierSubsystem(VisionPortal.Builder vpb){

        //webCam = wc;
        visionPortalBuilder = vpb;

    }


    public void setPosition(Positions.TEPosition position){
        Positions.getInstance().setTEPosition(position);
        localPosition = position;


    }


    public Positions.TEPosition getPosition(){

        return Positions.getInstance().getTEPosition();
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
        return Positions.getInstance().isPositionSet();
    }
}
