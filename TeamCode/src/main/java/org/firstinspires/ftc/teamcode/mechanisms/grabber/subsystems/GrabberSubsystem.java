package org.firstinspires.ftc.teamcode.mechanisms.grabber.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class GrabberSubsystem extends SubsystemBase {
    private ServoEx grabber;
    private double GRABBER_CLOSE_POSITION = 0.75;
    private double GRABBER_OPEN_POSITION = 1.0;

    public GrabberSubsystem(ServoEx grab){

        grabber = grab;
    }
    public void grab(double position){
        grabber.setPosition(position);
    }

    public void openGrabber(){
        grab(GRABBER_OPEN_POSITION);
    }
    public void closeGrabber(){
        grab(GRABBER_CLOSE_POSITION);
    }

    public double getPosition(){
        return grabber.getPosition();
    }
    public double getClosePosition(){
        return GRABBER_CLOSE_POSITION;
    }
    public double getOpenPosition(){
        return GRABBER_OPEN_POSITION;
    }
}
