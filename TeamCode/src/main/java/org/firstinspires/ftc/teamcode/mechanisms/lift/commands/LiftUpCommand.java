package org.firstinspires.ftc.teamcode.mechanisms.lift.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

//13 extended the base command
public class LiftUpCommand extends CommandBase {

    //14. Added a variable for the liftSubsystem (since the command needs to control the lift)
    private final LiftSubsystem liftSubsystem;
    //15. Added a variable for power (a supplier, provides a value without the variable having to poll for changes
    //in other scenarios, we would have to poll for trigger changes so we know how much to move the lift
    //by passing in a Supplier, in this case a Double, this power variable gets automatically updated)
    private DoubleSupplier power;
    //16. Who doesn't love telemetry
    private Telemetry telemetry;

    //17. Setup the constructors, I always have a base constructor and one that uses telemetry
    //given telemetry adds overhead we should do a better job about turning it off during competition
    public LiftUpCommand(LiftSubsystem liftSubsystem, DoubleSupplier power){
        //18. set our variables
        this.liftSubsystem = liftSubsystem;
        this.power = power;

        //19. set the requirements for this command
        //since commands are based on subsystems, each command should announce those dependencies to the rest of the system
        //ftclib handles the loop that an Op mode makes and handles the competition between commands that use the same
        //subsystems
        //https://docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system/command-scheduler
        //https://docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system/command
        addRequirements(liftSubsystem);
    }

    //19. Same as above except with Telemetry
    public LiftUpCommand(LiftSubsystem liftSubsystem, DoubleSupplier power, Telemetry telemetry){
        this.liftSubsystem = liftSubsystem;
        this.power = power;
        this.telemetry = telemetry;

        addRequirements(liftSubsystem);
    }

    //20. execute is one of the parts of the command lifecycle that gets called by the command scheduler
    //if a command just executes once and quits, then doing everything in the constructor is fine,
    //however, if the command is active and the scheduler needs  to keep checking it for action, the execute is the place
    @Override
    public void execute() {

        //21. Everything in here is just telemetry except when ask the motor to turn with the power of the trigger
        //and also throttling the speed for better control
        telemetry.addLine("Lift Executing");
        telemetry.update();
        liftSubsystem.turn(power.getAsDouble() * 0.8);
        telemetry.addData("Motor Power", liftSubsystem.getPower());
        telemetry.addData("Motor Max Level in Set Lift Right Cmd", liftSubsystem.getMaxTargetPosition());
        telemetry.addData("Motor Current Level in Set Lift Right Cmd", liftSubsystem.getCurrentPosition());
        telemetry.update();
    }

    //22. the end command gets called when isFinished (not pictured here), or when another command takes over the subsystem
    //based on the scheduler, so in this case, when the LiftDownCommand makes a request to control the lift, the lift will
    //stop and before it starts going down
    @Override
    public void end(boolean interupt){
        liftSubsystem.stopLift();
    }

   //23. next go to the liftdowncommand
}
