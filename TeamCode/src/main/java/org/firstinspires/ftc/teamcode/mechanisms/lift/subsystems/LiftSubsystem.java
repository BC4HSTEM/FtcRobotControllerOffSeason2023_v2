package org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {

    //1. Changed this to MotorEx from DcMotor (MotorEx is based on FTCLib)
    private MotorEx LM;

    //2. Added Telemetry as a best practice
    private Telemetry telemetry;
    //4. Added in direction variable
    private DcMotorEx.Direction direction;
    //5. Added in default power variable
    private double power = 0.0;
    //6. Added in the ability to throttle power
    private double powerRatio = 0.75;
    //7. Created a runmode variable (https://docs.ftclib.org/ftclib/v/v2.0.0/features/hardware/motors#using-a-runmode)
    private DcMotorEx.RunMode mode;
    //8. Added in Min and Max Target Positions based on last years numbers
    private static final int MIN_TARGET_POSITION = 0;
    private int maxTargetPosition = 4250;


    //3 Changed DataTypes from DcMotor to MotorEx
    public LiftSubsystem(MotorEx LiftMotor  ){
        LM = LiftMotor;
    }
    //9. Added another constructor that takes in Telemetry
    public LiftSubsystem(MotorEx l, Telemetry telemetry){

        LM = l;
        this.telemetry = telemetry;
    }

    //10. Added in some public setters for our variables above so commands can trigger them
    public void turn(double speed){
        setPower(speed*powerRatio);
    }

    public void stopResetEncoder(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    //// set the inversion factor
    //m_motor.setInverted(true);
    //
    //// get the inversion factor
    //boolean isInverted = m_motor.getInverted();
    //
    //// set the zero power behavior to BRAKE
    //m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    public void setZeroPowerBehavoir(DcMotorEx.ZeroPowerBehavior behavior){
        LM.motorEx.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotorEx.Direction direction){
        this.direction = direction;
        LM.motor.setDirection(this.direction);
    }

    public void setPower(double power){
        this.power = power;
        LM.set(this.power);
    }

    public void setMode(DcMotorEx.RunMode mode){
        this.mode = mode;
        LM.motorEx.setMode(this.mode);
    }

    public void setTargetPosition(int targetPosition){

        LM.motorEx.setTargetPosition(targetPosition);

    }

    public void setBrake(){
        setZeroPowerBehavoir(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //11. Added some public accessors to get values from the lift
    public int getTargetPosition(){
        return LM.motorEx.getTargetPosition();
    }

    public int getCurrentPosition(){
        return LM.getCurrentPosition();
    }

    public int getMinTargetPosition (){

        return MIN_TARGET_POSITION;
    }

    public int getMaxTargetPosition(){
        return maxTargetPosition;
    }

    public void stopLift(){
        LM.stopMotor();
    }

    //used if the motor is configured opposite of what's considered forward rotation
    public void setInverted(boolean inv){
        LM.setInverted(inv);
    }

    public boolean atTargetPosition() {
        return LM.motorEx.getTargetPosition() == LM.motorEx.getCurrentPosition();
    }


    public double getPower(){
        return LM.motorEx.getPower();
    }

    public DcMotorSimple.Direction getDirection(){
        return LM.motorEx.getDirection();
    }

    //12. go to the commands (LiftUpCommand)

}
