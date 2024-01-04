package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class HelloWorld extends OpMode {
    // Delete DcMotor liftMotor
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotorEx Arm = null;

    Servo PixelGrabberLeft = null;
    Servo PixelGrabberRight = null;
    // Setting up servos for drone launcher and wrist grabber
    Servo wristMotion = null;
    Servo droneLaunch =null;
    boolean grabberLeftOpen = true;
    boolean grabberRightOpen = true;

    public static int topPosition = 2100;


    public void init(){
        // Took out lift motor
        FrontLeft = hardwareMap.get(DcMotor.class,"FL");
        FrontRight = hardwareMap.get(DcMotor.class,"FR");
        BackLeft = hardwareMap.get(DcMotor.class,"BL");
        BackRight = hardwareMap.get(DcMotor.class,"BR");

        // Sydney Added the arm!
        Arm = hardwareMap.get(DcMotorEx.class,"arm" );
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);


        Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // THE RIGHT motors needed to be reversed
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection((DcMotorSimple.Direction.REVERSE));
        telemetry.addData("app started","hooray");
        telemetry.addData("arm","init");

        // Servos for Pixel Grabber
        PixelGrabberLeft = hardwareMap.get(Servo.class, "pixel_grabber_left");
        PixelGrabberRight = hardwareMap.get(Servo.class, "pixel_grabber_right");
        droneLaunch = hardwareMap.get(Servo.class,"drone_Launch");
        wristMotion = hardwareMap.get(Servo.class,"wrist_Motion");

        telemetry.addData("pixel grabber left", PixelGrabberLeft.getPosition());
        telemetry.addData("pixel grabber right", PixelGrabberRight.getPosition());
        telemetry.addData("drone_Launch", droneLaunch.getPosition());
        telemetry.addData("wrist_Motion", wristMotion.getPosition());

    }

    public void loop(){

        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5){
            telemetry.addData("drone_Launch", droneLaunch.getPosition());
        }

        if (gamepad1.left_stick_x != 0.0 || gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_x != 0 ){
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            FrontLeft.setPower(y + x + rx);
            BackLeft.setPower(y - x + rx);
            FrontRight.setPower(y - x - rx);
            BackRight.setPower(y + x - rx);
        }
        else {
            FrontLeft.setPower(0.0);
            BackRight.setPower(0.0);
            BackLeft.setPower(0.0);
            FrontRight.setPower(0.0);
        }

        if(gamepad1.y){
            Arm.setTargetPosition(topPosition);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.5);

        }else if(gamepad1.x){
            Arm.setTargetPosition(120);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.5);

        }else if(gamepad1.a){
            Arm.setTargetPosition(20);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.5);

        }else{
            Arm.setPower(0.0);
        }

        if (gamepad1.left_bumper){
            grabberLeftOpen = !grabberLeftOpen;
        }
        if (gamepad1.right_bumper){
            grabberRightOpen = !grabberRightOpen;
        }
        if(!gamepad1.left_bumper && grabberLeftOpen){
            PixelGrabberLeft.setPosition(0.3);
        }else{
            PixelGrabberLeft.setPosition(0.7);
        }

        if(!gamepad1.right_bumper && grabberRightOpen){
            PixelGrabberRight.setPosition(0.55);
        }else{
            PixelGrabberRight.setPosition(0.2);
        }


        telemetry.addData("pixel grabber left", PixelGrabberLeft.getPosition());
        telemetry.addData("pixel grabber right", PixelGrabberRight.getPosition());

        telemetry.addData("drone_Launch", droneLaunch.getPosition());
        telemetry.addData("wrist_Motion", wristMotion.getPosition());

        telemetry.addData("arm", Arm.getCurrentPosition());
        telemetry.update();



    }
}