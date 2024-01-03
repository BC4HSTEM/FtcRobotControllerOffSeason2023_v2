package org.firstinspires.ftc.teamcode.globals;

import java.util.HashMap;
import java.util.Map;

public enum Positions {
    INSTANCE;

    public enum TEPosition {
        POSITION_RIGHT,
        POSITION_MIDDLE,
        POSITION_LEFT,
        NONE
    }

    //Map<TEPosition, Integer> positions = new HashMap<>();


    Positions(){
        /*positions = new HashMap<>();
        positions.put(TEPosition.NONE,0);

        positions.put(TEPosition.POSITION_1,1);
        positions.put(TEPosition.POSITION_2,2);
        positions.put(TEPosition.POSITION_3,3);*/

        setTEPosition(TEPosition.POSITION_RIGHT);
    }
    private TEPosition selectedTEPosition = TEPosition.POSITION_RIGHT;

    public void setTEPosition(TEPosition position){
        selectedTEPosition = position;
    }

    public TEPosition getTEPosition(){
        return selectedTEPosition;
    }

    public static Positions getInstance(){
        return INSTANCE;
    }

}
