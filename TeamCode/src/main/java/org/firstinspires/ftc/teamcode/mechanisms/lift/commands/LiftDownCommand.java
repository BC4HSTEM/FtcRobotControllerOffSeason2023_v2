package org.firstinspires.ftc.teamcode.mechanisms.lift.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

//24. This is literally the same as LiftUpCommand except we pass in a negative value for power
//25. Go to CreateLiftMechanism
public class LiftDownCommand extends CommandBase{
    private LiftSubsystem liftSubsystem;
    private DoubleSupplier power;
    private Telemetry telemetry;

    public LiftDownCommand(LiftSubsystem liftSubsystem, DoubleSupplier power){
        this.liftSubsystem = liftSubsystem;
        this.power = power;

        addRequirements(liftSubsystem);
    }

    public LiftDownCommand(LiftSubsystem liftSubsystem, DoubleSupplier power, Telemetry telemetry){
        this.liftSubsystem = liftSubsystem;
        this.power = power;
        this.telemetry = telemetry;

        addRequirements(liftSubsystem);
    }


    @Override
    public void execute(){

        telemetry.addLine("LiftDown Executing");
        telemetry.update();
        liftSubsystem.turn(-power.getAsDouble()*0.5);
        telemetry.addData("Motor Power", liftSubsystem.getPower());
        telemetry.addData("Motor Min Level in Set Lift Left Cmd", liftSubsystem.getMinTargetPosition());
        telemetry.addData("Motor Current Level in Set Lift Left Cmd", liftSubsystem.getCurrentPosition());

        telemetry.update();
    }

    @Override
    public void end(boolean interupt){

        liftSubsystem.stopLift();

    }
}
