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
    public void grab(double rAngle, double lAngle){
        //pixelGrabberRight.turnToAngle(rAngle);
        //pixelGrabberLeft.turnToAngle(lAngle);
        telemetry.addData("right angle", pixelGrabberRight.getAngle() + rAngle);
        telemetry.addData("left angle", pixelGrabberLeft.getAngle() + lAngle);
        telemetry.update();
    }

    //6. Define functions that the commands can call
    public void openGrabber(){
        grab(GRABBER_RIGHT_OPEN_ANGLE, GRABBER_LEFT_OPEN_ANGLE);
    }
    public void closeGrabber(){
        grab(GRABBER_RIGHT_CLOSE_ANGLE, GRABBER_LEFT_CLOSE_ANGLE);
    }

    //7. Accessors for telemetry and isFinished in Commands
    public double getGrabberRightAngle(){
        return pixelGrabberRight.getAngle();
    }
    public double getGrabberLeftAngle(){
        return pixelGrabberLeft.getAngle();
    }
    /*public double getRightCloseAngle){
        return GRABBER_RIGHT_CLOSE_ANGLE;
    }
    public double getLeftCloseAngle){
        return GRABBER_LEFT_CLOSE_ANGLE;
    }
    public double getRightOpenAngle(){
        return GRABBER_RIGHT_OPEN_ANGLE;
    }

    public double getLeftOpenAngle(){
        return GRABBER_LEFT_OPEN_ANGLE;
    }*/

    //8. go to GrabberCommand
}
