package org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;
    private MecanumDrive drive;
    private Motor fl, fr, bl, br;
    private boolean slowmode = false;
    public DriveSubsystem(Motor frontL, Motor frontR, Motor backL, Motor backR){
                    fl = frontL;
                    fr = frontR;
                    bl = backL;
                    br = backR;
                    drive = new MecanumDrive(fl, fr, bl, br);
    }
    public void drive(double strafe, double forward, double turn){
        if(slowmode){
            drive.driveRobotCentric(-strafe, -forward, turn);
        }
        else{
            drive.driveRobotCentric(-strafe, -forward, -turn);
        }
    }

    public Motor getFl(){
        return fl;
    }
    public Motor getFr(){
        return fr;
    }
    public Motor getBl(){
        return bl;
    }
    public Motor getBr(){
        return br;
    }


}
