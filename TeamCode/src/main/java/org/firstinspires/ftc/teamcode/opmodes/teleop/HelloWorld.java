package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HelloWorld extends OpMode {
    DcMotor LiftMotor = null;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotorEx Arm = null;

    Servo PixelGrabberLeft = null;
    Servo PixelGrabberRight = null;
<<<<<<< HEAD

    Boolean grabberLeftOpen = true;
    Boolean grabberRightOpen = true;

=======
    boolean grabberLeftOpen = true;
    boolean grabberRightOpen = true;
>>>>>>> 6d25f846f3cbc1e2df86c641f792dc6f6deb5712

    public void init(){
        LiftMotor = hardwareMap.get(DcMotor.class,"LM");
        FrontLeft = hardwareMap.get(DcMotor.class,"FL");
        FrontRight = hardwareMap.get(DcMotor.class,"FR");
        BackLeft = hardwareMap.get(DcMotor.class,"BL");
        BackRight = hardwareMap.get(DcMotor.class,"BR");

        // Sydney Added the arm!
        Arm = hardwareMap.get(DcMotorEx.class,"arm" );
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(20);


        Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // THE RIGHT motors needed to be reversed
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection((DcMotorSimple.Direction.REVERSE));
        telemetry.addData("app started","hooray");
        telemetry.addData("arm","init");

        // Servos for Pixel Grabber
        PixelGrabberLeft = hardwareMap.get(Servo.class, "pixel_grabber_left");
        PixelGrabberRight = hardwareMap.get(Servo.class, "pixel_grabber_right");


        telemetry.addData("pixel grabber left", PixelGrabberLeft.getPosition());
        telemetry.addData("pixel grabber right", PixelGrabberRight.getPosition());


    }

    public void loop(){

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
        if(gamepad1.left_trigger != 0.0){
            LiftMotor.setPower(gamepad1.left_trigger);

        }
        else{
            LiftMotor.setPower(0.0);
        }
        if(gamepad1.right_trigger != 0.0){
            LiftMotor.setPower(-gamepad1.right_trigger);

        }
        else{
            LiftMotor.setPower(0.0);
        }


        if(gamepad1.right_bumper ){
            Arm.setTargetPosition(700);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.5);

        }else if(gamepad1.left_bumper ){
            Arm.setTargetPosition(20);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.5);
            

        }else{
            Arm.setPower(0.0);
        }

        if (gamepad1.a){
            grabberLeftOpen = !grabberLeftOpen;
        }
        

        if(grabberLeftOpen){
            PixelGrabberLeft.setPosition(0.3);

        }else{
            PixelGrabberLeft.setPosition(0.7);

        }


        if(gamepad1.b){
            PixelGrabberRight.setPosition(0.55);
        }else{
            PixelGrabberRight.setPosition(0.2);
        }


        telemetry.addData("pixel grabber left", PixelGrabberLeft.getPosition());
        telemetry.addData("pixel grabber right", PixelGrabberRight.getPosition());

        telemetry.addData("arm", Arm.getCurrentPosition());
        telemetry.update();



    }
}