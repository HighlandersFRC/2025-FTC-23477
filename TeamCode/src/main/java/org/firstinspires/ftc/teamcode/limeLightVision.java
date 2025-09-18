package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


public class limeLightVision {


    public void runOpMode() throws InterruptedException {

        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            int id = result.getPipelineIndex();


            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.addData("ID information", id);



        }

    }
}
