package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;

@TeleOp
public class TestRangeTrackDetection extends LinearOpMode {

    AprilTagState aprilTagStates = new AprilTagState("aprilTagState");

    @Override
    public void runOpMode() throws InterruptedException {

        aprilTagStates.init(hardwareMap);

        waitForStart();

        boolean tracking = false;

        while (opModeIsActive()) {

            // Toggle range track with A
            if (gamepad1.a) {
                tracking = !tracking;
                sleep(250);
            }

            if (tracking) {
                aprilTagStates.setWantedState(AprilTagState.APRIL_TAG_STATE.RANGE_TRACK);
            } else {
                aprilTagStates.setWantedState(AprilTagState.APRIL_TAG_STATE.IDLE);
            }

            aprilTagStates.periodic();
        }
    }
}
