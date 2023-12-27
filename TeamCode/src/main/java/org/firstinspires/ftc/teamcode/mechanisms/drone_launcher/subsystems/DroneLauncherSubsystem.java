package org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DroneLauncherSubsystem extends SubsystemBase {

    private ServoEx droneLauncher;
    Telemetry telemetry;

    public static double launchPosition = 0;


    public DroneLauncherSubsystem(ServoEx dl, Telemetry telemetry, boolean useDB){

        droneLauncher = dl;

        if (useDB){
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        else{
            this.telemetry = telemetry;
        }
    }

    public void LaunchDrone(){
        telemetry.addLine("drone launch initialize");
        droneLauncher.setPosition(launchPosition);

        telemetry.addData("Launcher position", droneLauncher.getPosition());
        telemetry.update();
    }

    public double getDroneLauncherPosition(){
        return droneLauncher.getPosition();
    }
}
