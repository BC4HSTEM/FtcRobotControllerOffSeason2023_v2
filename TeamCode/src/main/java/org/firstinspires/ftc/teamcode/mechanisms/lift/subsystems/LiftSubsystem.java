package org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class LiftSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;
    private DcMotor LM;
    //private boolean slowmode = false;
    public LiftSubsystem(DcMotor LiftMotor  ){
        LM = LiftMotor;
    }
    public void lift(){

    }

    public DcMotor getLM(){
        return LM;
    }

}
