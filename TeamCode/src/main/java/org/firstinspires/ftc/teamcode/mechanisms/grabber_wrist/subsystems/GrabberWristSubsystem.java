package org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class GrabberWristSubsystem extends SubsystemBase {
    private ServoEx grabberWrist;
    Telemetry telemetry;

    public static double dropPosition = .5;
    public static double pickUpPosition = .27;


    public GrabberWristSubsystem(ServoEx gw, Telemetry telemetry, boolean useDB){

        grabberWrist = gw;

        if (useDB){
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        else{
            this.telemetry = telemetry;
        }
    }

    public void DropPixel(){
        telemetry.addLine("grabber wrist initialize");
        grabberWrist.setPosition(dropPosition);

        telemetry.addData("Grabber wrist drop position", grabberWrist.getPosition());
        telemetry.update();
    }

    public void PickUpPixel(){
        telemetry.addLine("grabber wrist initialize");
        grabberWrist.setPosition(pickUpPosition);

        telemetry.addData("grabber wrist pickup position", grabberWrist.getPosition());
        telemetry.update();
    }

    public void setInverted(boolean invert){
        grabberWrist.setInverted(invert);
    }

    public double getGrabberWristPosition(){
        return grabberWrist.getPosition();
    }
}
