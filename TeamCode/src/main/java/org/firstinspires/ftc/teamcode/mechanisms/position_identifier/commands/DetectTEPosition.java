package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;
import org.openftc.easyopencv.OpenCvPipeline;
public class DetectTEPosition extends CommandBase {

    private final PositionIdentifierSubsystem positionIdentifierSubsystem;

    private Telemetry telemetry;
    private Positions.TEPosition position;

    public DetectTEPosition(PositionIdentifierSubsystem subsystem){
        positionIdentifierSubsystem = subsystem;
        addRequirements(positionIdentifierSubsystem);
    }

    public DetectTEPosition(PositionIdentifierSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        positionIdentifierSubsystem = subsystem;
        this.telemetry = telemetry;

        addRequirements(positionIdentifierSubsystem);
    }

    @Override
    public void initialize(){

        positionIdentifierSubsystem.openCameraDeviceAsync();
        //telemetry.addData("We are initialize", "detectPosition");
    }
}
