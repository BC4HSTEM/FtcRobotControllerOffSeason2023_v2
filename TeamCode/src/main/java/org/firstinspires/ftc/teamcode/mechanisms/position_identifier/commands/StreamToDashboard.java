package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;

public class StreamToDashboard extends CommandBase {
    private final PositionIdentifierSubsystem positionIdentifierSubsystem;

    private final FtcDashboard dashboard;


    public StreamToDashboard(PositionIdentifierSubsystem subsystem){
        positionIdentifierSubsystem = subsystem;
        dashboard = null;

    }

    public StreamToDashboard(PositionIdentifierSubsystem subsystem, FtcDashboard dashboard){
        positionIdentifierSubsystem = subsystem;
        this.dashboard = dashboard;

        addRequirements(positionIdentifierSubsystem);
    }

    @Override
    public void initialize(){
        positionIdentifierSubsystem.StreamToDashboard(dashboard);
    }

}
