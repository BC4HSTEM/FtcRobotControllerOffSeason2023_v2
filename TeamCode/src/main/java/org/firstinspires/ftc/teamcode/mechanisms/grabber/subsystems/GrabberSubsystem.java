package org.firstinspires.ftc.teamcode.mechanisms.grabber.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

//1. Extend Subsystem base class
public class GrabberSubsystem extends SubsystemBase {
    //2. setup a SeveroEx variable
    private ServoEx grabber;
    //3. Define the open and close position of the grabber
    private double GRABBER_CLOSE_POSITION = 0.75;
    private double GRABBER_OPEN_POSITION = 1.0;

    //4. Define you constructor .... we should probably have one with telemetry passed to it
    public GrabberSubsystem(ServoEx grab){

        grabber = grab;
    }
    //5. define a grab function that sets the servo position....this function should probably be private
    public void grab(double position){
        grabber.setPosition(position);
    }

    //6. Define functions that the commands can call
    public void openGrabber(){
        grab(GRABBER_OPEN_POSITION);
    }
    public void closeGrabber(){
        grab(GRABBER_CLOSE_POSITION);
    }

    //7. Accessors for telemetry and isFinished in Commands
    public double getPosition(){
        return grabber.getPosition();
    }
    public double getClosePosition(){
        return GRABBER_CLOSE_POSITION;
    }
    public double getOpenPosition(){
        return GRABBER_OPEN_POSITION;
    }

    //8. go to GrabberCommand
}
