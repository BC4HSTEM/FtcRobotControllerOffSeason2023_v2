package org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//1. Extend Subsystem base class
@Config
public class PixelGrabberSubsystem extends SubsystemBase {
    //2. setup a SeveroEx variable
    private ServoEx pixelGrabberRight;
    private ServoEx pixelGrabberLeft;

    private Telemetry telemetry;
    //3. Define the open and close position of the grabber
    public static double GRABBER_RIGHT_CLOSE_ANGLE = 100;
    public static double GRABBER_LEFT_CLOSE_ANGLE = 150;
    public static double GRABBER_RIGHT_OPEN_ANGLE = 150;
    public static double GRABBER_LEFT_OPEN_ANGLE = 100;

    public static double GRABBER_RIGHT_CLOSE_POSITION = 0.7;
    public static double GRABBER_LEFT_CLOSE_POSITION = 0.7;
    public static double GRABBER_RIGHT_OPEN_POSITION = 0.3;
    public static double GRABBER_LEFT_OPEN_POSITION = 0.3;

    //4. Define you constructor .... we should probably have one with telemetry passed to it
    public PixelGrabberSubsystem(ServoEx grabberRight, ServoEx grabberLeft){

        pixelGrabberRight = grabberRight;
        pixelGrabberLeft = grabberLeft;
    }

    public PixelGrabberSubsystem(ServoEx grabberRight, ServoEx grabberLeft, Telemetry telemetry, boolean useDB){

        pixelGrabberRight = grabberRight;
        pixelGrabberLeft = grabberLeft;
        if (useDB){
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        else{
            this.telemetry = telemetry;
        }
    }
    //5. define a grab function that sets the servo position....this function should probably be private
    public void grabRight(double rAngle){
        pixelGrabberRight.turnToAngle(rAngle);
    }

    public void grabLeft(double lAngle){
        pixelGrabberLeft.turnToAngle(lAngle);
    }
    public void grabRightPosition(double rPosition){
        pixelGrabberRight.setPosition(rPosition);
    }

    public void grabLeftPosition(double lPosition){
        pixelGrabberLeft.setPosition(lPosition);
    }

    //6. Define functions that the commands can call
    public void openRightGrabber(){
        grabRight(GRABBER_RIGHT_OPEN_ANGLE);
    }
    public void closeRightGrabber(){
        grabRight(GRABBER_RIGHT_CLOSE_ANGLE);
    }

    public void openLeftGrabber(){
        grabLeft(GRABBER_LEFT_OPEN_ANGLE);
    }
    public void closeLeftGrabber(){
        grabLeft(GRABBER_LEFT_CLOSE_ANGLE);
    }
    public void openRightGrabberPosition(){
        grabRightPosition(GRABBER_RIGHT_OPEN_POSITION);
    }
    public void closeRightGrabberPosition(){
        grabRightPosition(GRABBER_RIGHT_CLOSE_POSITION);
    }

    public void openLeftGrabberPosition(){
        grabLeftPosition(GRABBER_LEFT_OPEN_POSITION);
    }
    public void closeLeftGrabberPosition(){
        grabLeftPosition(GRABBER_LEFT_CLOSE_POSITION);
    }
    //7. Accessors for telemetry and isFinished in Commands
    public double getGrabberRightAngle(){
        return pixelGrabberRight.getAngle();
    }
    public double getGrabberLeftAngle(){
        return pixelGrabberLeft.getAngle();
    }
    public double getGrabberRightPosition(){
        return pixelGrabberRight.getPosition();
    }
    public double getGrabberLeftPosition(){
        return pixelGrabberLeft.getPosition();
    }

    /*public double getRightCloseAngle){
        return GRABBER_RIGHT_CLOSE_ANGLE;
    }*/
    /*public double getLeftCloseAngle){
        return GRABBER_LEFT_CLOSE_ANGLE;
    }*/
    public double getLeftClosePosition(){
        return GRABBER_LEFT_CLOSE_POSITION;
    }
    public double getRightOpenAngle(){
        return GRABBER_RIGHT_OPEN_ANGLE;
    }

    public double getLeftOpenAngle(){
        return GRABBER_LEFT_OPEN_ANGLE;
    }

    public double getLeftOpenPosition(){
        return GRABBER_LEFT_OPEN_POSITION;
    }

    //8. go to GrabberCommand
}
