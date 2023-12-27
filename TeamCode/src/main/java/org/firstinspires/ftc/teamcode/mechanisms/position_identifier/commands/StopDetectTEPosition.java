package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;
public class StopDetectTEPosition extends CommandBase {
    private final PositionIdentifierSubsystem positionIdentifierSubsystem;
    private Telemetry telemetry;
    private int position = 0;

    public StopDetectTEPosition(PositionIdentifierSubsystem subsystem){
        positionIdentifierSubsystem = subsystem;

        addRequirements(positionIdentifierSubsystem);
    }

    public StopDetectTEPosition(PositionIdentifierSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        positionIdentifierSubsystem = subsystem;
        this.telemetry = telemetry;

        addRequirements(positionIdentifierSubsystem);
    }

    @Override
    public void initialize(){
        telemetry.addLine("stopping the stream at level: " + positionIdentifierSubsystem.getPosition());
        positionIdentifierSubsystem.stopStreaming();
    }



    @Override
    public boolean isFinished() {
        //telemetry.addData("We are finished", gotPosition);
        //telemetry.update();
        return true;
    }
}
