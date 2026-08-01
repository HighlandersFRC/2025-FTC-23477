package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem;

@Autonomous
public class ninetyDegreeSubsystemTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem subsystem = new driveSubsystem("subsystem");
        subsystem.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            subsystem.setWantedState(driveSubsystem.rotateStates.CLOCKWISE_TURN);
            subsystem.periodic();
        }
    }
}
