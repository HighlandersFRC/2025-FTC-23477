package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.commands.commandDriveBack;
import org.firstinspires.ftc.teamcode.commands.commandIndex;
import org.firstinspires.ftc.teamcode.commands.commandScheduler;
import org.firstinspires.ftc.teamcode.commands.commandShoot;
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem;

@Autonomous
public class shoot3Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        driveSubsystem rotate = new driveSubsystem("rotate");
        commandScheduler scheduler = new commandScheduler();

        double current_x = rotate.getXTraveled();
        double current_y = rotate.getYTraveled();
        double heading = rotate.getCurrentHeading();

        waitForStart();
        while (opModeIsActive()) {
            scheduler.schedule(new commandDriveBack(30, hardwareMap));
            scheduler.schedule(new ParallelCommandGroup(
                scheduler,
                Parameters.ALL,
                new commandIndex(hardwareMap, 15),
                new commandShoot(hardwareMap, 15)
            ));
            scheduler.run();

            telemetry.addData("X:", current_x);
            telemetry.addData("Y:", current_y);
            telemetry.addData("Heading:", heading);
            telemetry.update();
        }
    }
}
