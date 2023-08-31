package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpTest extends OpMode {

    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    public void init(){
        FrontLeft = hardwareMap.get(DcMotor.class,"FL");
        FrontRight = hardwareMap.get(DcMotor.class,"Fr");
        BackLeft = hardwareMap.get(DcMotor.class,"BL");
        BackRight = hardwareMap.get(DcMotor.class,"BR");

        FrontLeft = hardwareMap.get(DcMotorSimple.Direction.REVERSE);
        BackLeft = hardwareMap.get(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){
        if(gamepad1.left_stick_y != 0.0){
            FrontLeft.setPower(~gamepad1.left_stick_y);
            FrontRight.setPower(~gamepad1.left_stick_y);
            BackLeft.setPower(~gamepad1.left_stick_y);
            BackRight.setPower(~gamepad1.left_stick_y);
        }
        else{
            FrontLeft.setPower(0.0);
            FrontRight.setPower(0.0);
            BackLeft.setPower(0.0);
            BackLeft.setPower(0.0);
        }
    }
}
