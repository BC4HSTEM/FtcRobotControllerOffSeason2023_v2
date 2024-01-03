package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

public enum PixelTrajectory {
    INSTANCE;

    Trajectory pixelTraj;



    public void setPixelTraj(Trajectory pt){
        pixelTraj = pt;
    }

    public Trajectory getPixelTraj(){
        return pixelTraj;
    }

    public static PixelTrajectory getInstance(){
        return INSTANCE;
    }
}