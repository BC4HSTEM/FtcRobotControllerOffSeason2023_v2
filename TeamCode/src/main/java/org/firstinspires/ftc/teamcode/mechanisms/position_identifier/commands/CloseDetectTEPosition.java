package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;

public class CloseDetectTEPosition extends CommandBase {

    private final PositionIdentifierSubsystem positionIdentifierSubsystem;

    private Telemetry telemetry;
    private int level = 0;

    public CloseDetectTEPosition(PositionIdentifierSubsystem subsystem){
        positionIdentifierSubsystem = subsystem;

        addRequirements(subsystem);
    }

    public CloseDetectTEPosition(PositionIdentifierSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        positionIdentifierSubsystem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        telemetry.addLine("stopping the stream at level: " + positionIdentifierSubsystem.getPosition());
        positionIdentifierSubsystem.closeStream();
    }



    @Override
    public boolean isFinished() {
        return true;
    }

}
