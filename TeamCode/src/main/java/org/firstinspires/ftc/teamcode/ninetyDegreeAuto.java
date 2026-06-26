package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tools.robot;
import org.firstinspires.ftc.teamcode.commands.commandClockwise;
import org.firstinspires.ftc.teamcode.commands.commandScheduler;

@Autonomous
public class ninetyDegreeAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        commandScheduler scheduler = new commandScheduler();
        robot robot = new robot(hardwareMap);
        robot.initialize(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            scheduler.schedule(new commandClockwise(hardwareMap));
            scheduler.run();
        }
    }
}
