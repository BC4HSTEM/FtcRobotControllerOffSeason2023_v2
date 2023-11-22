package org.firstinspires.ftc.teamcode.mechanisms.arm.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems.ArmSubsystem;

@Config
public class ArmPickUpCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private Telemetry telemetry;

    public static int pickUpTargetPosition = 50;

    public ArmPickUpCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    public ArmPickUpCommand(ArmSubsystem armSubsystem, Telemetry telemetry){
        this.armSubsystem = armSubsystem;
        this.telemetry = telemetry;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setTargetPosition(pickUpTargetPosition);
    }
}
