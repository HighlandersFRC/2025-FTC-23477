package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class limeLightVision extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                String botPose = result.getBotpose().toString();

                double pipeline = result.getPipelineIndex();
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                int tagcount = result.getBotposeTagCount();

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.addData("Tag Count", tagcount);
                telemetry.addData("Information", result.toString());
                telemetry.addData("Bot Position", botPose);
                telemetry.addData("Pipeline", pipeline);
                telemetry.update();
            }
        }
    }
}
