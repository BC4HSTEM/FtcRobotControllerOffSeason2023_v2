package org.firstinspires.ftc.teamcode.globals;

public enum Path {
    INSTANCE;

    public enum PositionToFollow {
        PATH_1,
        PATH_2,
        PATH_3,
        PATH_4
    }

    private PositionToFollow selectedPathToFollow = PositionToFollow.PATH_1;

    public void setSelectedPathToFollow(PositionToFollow positionSide){
        selectedPathToFollow = positionSide;
    }

    public PositionToFollow getSelectedPathToFollow(){
        return selectedPathToFollow;
    }

    public static Path getInstance(){
        return INSTANCE;
    }
}