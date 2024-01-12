package org.firstinspires.ftc.teamcode.mechanisms.position_identifier;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Side;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.CloseDetectTEPosition;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.DetectTEPosition;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.StopDetectTEPosition;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.StreamToDashboard;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.TSEProcessorLeft;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.TSEProcessorRight;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

public class CreatePositionIdentifierMechanism extends CreateMechanismBase {

    private  PositionIdentifierSubsystem positionIdentifierSubsystem;

    private FtcDashboard dashboard;

    private DetectTEPosition detectTEPosition;
    private StopDetectTEPosition stopDetectTEPosition;
    private CloseDetectTEPosition closeDetectTEPosition;
    //private MockDetectTSEPosition mockDetectTSEPosition;
    private StreamToDashboard streamToDashboard;
    //private SetArmLevel setArmLevel;

    private OpenCvWebcam webCam;

    private int width = 320;
    private int height = 240;


    public CreatePositionIdentifierMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);
        this.dashboard = FtcDashboard.getInstance();

    }

    //24. Constructor for Teleop .... notice the GamepdEx variable
    public CreatePositionIdentifierMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);
        this.dashboard = FtcDashboard.getInstance();

    }

    //25. Constructor for Autonomous .... notice there is no GamePadEx variable
    public CreatePositionIdentifierMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);
        this.dashboard = FtcDashboard.getInstance();

    }


    public void create(){

        createBase();
        //setArmLevel = new SetArmLevel(armSubsystem,armSubsystem.getLevel(subsystem.getLevel()),telemetry);
        op.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(detectTEPosition);
        //detectTEPosition.schedule();


        //CommandScheduler.getInstance().onCommandFinish( detectTSEPosition -> telemetry.addData("got position", subsystem.getLocation()));

    }

    public void createAuto(){
        //subsystem = new WebCamSubsystem(hwMap,deviceName,new OpenCvShippingElementDetector(320,240,telemetry));
        createBase();


    }
    @Override
    public void createBase(){

        VisionProcessor tseProcessor = null;

        if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
            tseProcessor = new TSEProcessorLeft(telemetry);
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
            tseProcessor = new TSEProcessorRight(telemetry);
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
            tseProcessor = new TSEProcessorRight(telemetry);
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
            tseProcessor = new TSEProcessorLeft(telemetry);
        }
        VisionPortal.Builder myVisionPortalBuilder;


// Create a new VisionPortal Builder object.
        myVisionPortalBuilder = new VisionPortal.Builder();

// Specify the camera to be used for this VisionPortal.
        myVisionPortalBuilder.setCamera(hwMap.get(WebcamName.class, deviceName));      // Other choices are: RC phone camera and "switchable camera name".

// Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(tseProcessor);       // An added Processor is enabled by default.

// Optional: set other custom features of the VisionPortal (4 are shown here).
        myVisionPortalBuilder.setCameraResolution(new Size(width, height));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

// Create a VisionPortal by calling build()


        positionIdentifierSubsystem = new PositionIdentifierSubsystem(myVisionPortalBuilder);

        detectTEPosition = new DetectTEPosition(positionIdentifierSubsystem, telemetry);
        stopDetectTEPosition = new StopDetectTEPosition(positionIdentifierSubsystem,telemetry);
        closeDetectTEPosition = new CloseDetectTEPosition(positionIdentifierSubsystem,telemetry);

    }

    public DetectTEPosition getDetectTEPositionCommand(){
        return detectTEPosition;

    }

    public StopDetectTEPosition getStopDetectTEPosition(){
        return stopDetectTEPosition;
    }
    public CloseDetectTEPosition getCloseDetectTEPosition(){
        return closeDetectTEPosition;
    }


    public PositionIdentifierSubsystem getPositionIdentifierSubsystem(){
        return positionIdentifierSubsystem;
    }

    public boolean isPositionSet(){
        return positionIdentifierSubsystem.isPositionSet();
    }
}
