package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.CameraStates;

@TeleOp
public class Testing123 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CameraStates drive = new CameraStates("cam123");
        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drive.periodic();

            drive.setWantedState(CameraStates.CAMERA_STATES.AUTO_TARGET);
            if (drive.isFinished()) {
                drive.setWantedState(CameraStates.CAMERA_STATES.DEFAULT);
            }
        }
    }
}
