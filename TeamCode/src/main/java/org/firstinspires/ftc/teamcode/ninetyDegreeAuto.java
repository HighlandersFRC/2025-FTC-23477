package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.commandClockwise;
import org.firstinspires.ftc.teamcode.commands.commandScheduler;

@Autonomous
public class ninetyDegreeAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        commandScheduler scheduler = new commandScheduler();

        waitForStart();
        while (opModeIsActive()) {
            scheduler.schedule(new commandClockwise(hardwareMap));
            scheduler.run();
        }
    }
}
