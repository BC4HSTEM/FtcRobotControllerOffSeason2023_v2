package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.TSEProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class PositionIdentifierSubsystem extends SubsystemBase {


    private VisionPortal.Builder visionPortalBuilder;
    private VisionPortal visionPortal;


    public PositionIdentifierSubsystem(VisionPortal.Builder vpb){

        //webCam = wc;
        visionPortalBuilder = vpb;

    }


    public void setPosition(Positions.TEPosition position){
        Positions.getInstance().setTEPosition(position);

    }


    public Positions.TEPosition getPosition(){
        return Positions.getInstance().getTEPosition();
    }


    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    public void closeStream(){
        visionPortal.close();
    }

    public void detectTEPosition(){
        visionPortal = visionPortalBuilder.build();
    }
}
