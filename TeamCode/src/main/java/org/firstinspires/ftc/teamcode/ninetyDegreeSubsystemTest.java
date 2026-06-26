package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.rotateSubsystem;

@Autonomous
public class ninetyDegreeSubsystemTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        rotateSubsystem subsystem = new rotateSubsystem("subsystem");
        subsystem.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            subsystem.setWantedState(rotateSubsystem.rotateStates.CLOCKWISE_TURN);
            subsystem.periodic();
        }
    }
}
