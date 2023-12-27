package org.firstinspires.ftc.teamcode.mechanisms.position_identifier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.CloseDetectTEPosition;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.DetectTEPosition;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.StopDetectTEPosition;
//import org.firstinspires.ftc.teamcode.commands.webcam.StreamToDashboard;

import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.StreamToDashboard;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.ContourPipeline320w240h;


import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.function.Consumer;

public class CreatePositionIdentifierMechanism extends CreateMechanismBase {

    private Telemetry telemetry;
    private  PositionIdentifierSubsystem positionIdentifierSubsystem;

    private FtcDashboard dashboard;

    private DetectTEPosition detectTEPosition;
    private StopDetectTEPosition stopDetectTEPosition;
    private CloseDetectTEPosition closeDetectTEPosition;
    //private MockDetectTSEPosition mockDetectTSEPosition;
    private StreamToDashboard streamToDashboard;
    //private SetArmLevel setArmLevel;

    private OpenCvWebcam webCam;

    private static final String ID_NAME = "cameraMonitorViewId";
    private static final String ID_DEF_TYPE = "id";

    private Trigger gotPositionTrigger;

    public CreatePositionIdentifierMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, final FtcDashboard dashboard){
        super(hwMap, deviceName, op, telemetry);
        this.dashboard = dashboard;

    }

    //24. Constructor for Teleop .... notice the GamepdEx variable
    public CreatePositionIdentifierMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, final FtcDashboard dashboard, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);
        this.dashboard = dashboard;

    }

    //25. Constructor for Autonomous .... notice there is no GamePadEx variable
    public CreatePositionIdentifierMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry, final FtcDashboard dashboard){
        super(hwMap, deviceName, telemetry);
        this.dashboard = dashboard;

    }


    public void create(){

        createBase();
        //setArmLevel = new SetArmLevel(armSubsystem,armSubsystem.getLevel(subsystem.getLevel()),telemetry);

        detectTEPosition.schedule();


        //CommandScheduler.getInstance().onCommandFinish( detectTSEPosition -> telemetry.addData("got position", subsystem.getLocation()));

    }

    public void createAuto(){
        //subsystem = new WebCamSubsystem(hwMap,deviceName,new OpenCvShippingElementDetector(320,240,telemetry));
        createBase();
        op.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(detectTEPosition);

        //mockDetectTSEPosition = new MockDetectTSEPosition(subsystem, telemetry);




        //gotPositionTrigger = new Trigger(()->positionIdentifierSubsystem.getLevel() > 0);


    }
    @Override
    public void createBase(){
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier(ID_NAME, ID_DEF_TYPE, hwMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, deviceName), cameraMonitorViewId);

        positionIdentifierSubsystem = new PositionIdentifierSubsystem(webCam,new ContourPipeline320w240h(telemetry));

        detectTEPosition = new DetectTEPosition(positionIdentifierSubsystem, telemetry);
        stopDetectTEPosition = new StopDetectTEPosition(positionIdentifierSubsystem,telemetry);
        closeDetectTEPosition = new CloseDetectTEPosition(positionIdentifierSubsystem,telemetry);
        streamToDashboard = new StreamToDashboard(positionIdentifierSubsystem,dashboard);
        streamToDashboard.schedule();
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

    /*public MockDetectTSEPosition getMockDetectTSEPositionCommand(){
        return mockDetectTSEPosition;

    }*/

    public Trigger getGotPositionTrigger(){
        return gotPositionTrigger;
    }

    public PositionIdentifierSubsystem getPositionIdentifierSubsystem(){
        return positionIdentifierSubsystem;
    }
}
