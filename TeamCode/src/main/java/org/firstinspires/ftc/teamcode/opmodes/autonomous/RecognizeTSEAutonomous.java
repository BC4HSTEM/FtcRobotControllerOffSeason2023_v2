package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.pipelines.TSEProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous()
public class RecognizeTSEAutonomous extends OpMode {

    private TSEProcessor tseProcessor;

    private VisionPortal visionPortal;

    @Override
    public void init() {
        tseProcessor = new TSEProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaul0ts(hardwareMap.get(WebcamName.class, "Webcam 1"), tseProcessor);
    }
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Identified", tseProcessor.getSelection());
    }
}
