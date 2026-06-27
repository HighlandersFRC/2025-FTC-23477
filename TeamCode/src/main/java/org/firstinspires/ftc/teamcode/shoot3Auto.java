package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.commandDriveBack;
import org.firstinspires.ftc.teamcode.commands.commandIndex;
import org.firstinspires.ftc.teamcode.commands.commandScheduler;
import org.firstinspires.ftc.teamcode.commands.commandShoot;
import org.firstinspires.ftc.teamcode.subsystems.rotateSubsystem;

@Autonomous
public class shoot3Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        rotateSubsystem rotate = new rotateSubsystem("rotate");
        commandScheduler scheduler = new commandScheduler();

        double current_x = rotate.get_x_traveled();
        double current_y = rotate.get_y_traveled();
        double heading = rotate.get_current_heading();

        waitForStart();
        while (opModeIsActive()) {
            scheduler.schedule(new commandDriveBack(30, hardwareMap));
            scheduler.schedule(new commandIndex(hardwareMap, 15));
            scheduler.schedule(new commandShoot(hardwareMap, 15));
            scheduler.run();

            telemetry.addData("X:", current_x);
            telemetry.addData("Y:", current_y);
            telemetry.addData("Heading:", heading);
            telemetry.update();
        }
    }
}
