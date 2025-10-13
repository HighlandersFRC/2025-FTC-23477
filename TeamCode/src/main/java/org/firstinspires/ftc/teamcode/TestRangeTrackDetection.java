package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandRangeTrack;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

public class TestRangeTrackDetection extends LinearOpMode {
    AprilTagState aprilTagStates = new AprilTagState("aprilTagState");
    CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagStates.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.aprilTagState = aprilTagStates;
        scheduler.setNewRobot(robot);
        waitForStart();
        while (opModeIsActive()) {
            aprilTagStates.periodic();

            if (gamepad1.a) {
                aprilTagStates.setWantedState(AprilTagState.APRIL_TAG_STATE.RANGE_TRACK);
            } else if (gamepad1.b) {
                scheduler.schedule(new CommandRangeTrack(robot.aprilTagState));
            }
        }
    }

}
