package org.firstinspires.ftc.teamcode.globals;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

public enum Alliance {
    INSTANCE;

    private static Map<AllianceTeam, String> ALLIANCE_OPTIONS;

    public enum AllianceTeam {
        RED,
        BLUE
    }
    static {
        ALLIANCE_OPTIONS = new LinkedHashMap<>(); // Diamond operator requires Java 1.7+
        ALLIANCE_OPTIONS.put(AllianceTeam.BLUE,"Blue");
        ALLIANCE_OPTIONS.put(AllianceTeam.RED,"Red");
    }

    private AllianceTeam selectedAllianceTeam = AllianceTeam.RED;

    public void setAllianceTeam(AllianceTeam allianceTeam){
        selectedAllianceTeam = allianceTeam;
    }

    public AllianceTeam getAllianceTeam(){
        return selectedAllianceTeam;
    }

    public String getAllianceTeamOption(){

        return ALLIANCE_OPTIONS.get(this.selectedAllianceTeam);
    }

    public static Alliance getInstance(){
        return INSTANCE;
    }
}
