package org.firstinspires.ftc.teamcode.globals;

public enum Side {
    INSTANCE;

    public enum PositionSide {
        STAGE_SIDE,
        NON_STAGE_SIDE
    }

    private PositionSide selectedPositionSide = PositionSide.NON_STAGE_SIDE;

    public void setPositionSide(PositionSide positionSide){
        selectedPositionSide = positionSide;
    }

    public PositionSide getPositionSide(){
        return selectedPositionSide;
    }

    public static Side getInstance(){
        return INSTANCE;
    }
}