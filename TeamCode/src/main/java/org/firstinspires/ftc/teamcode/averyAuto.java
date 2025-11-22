package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnRight;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

@Autonomous
public class averyAuto extends LinearOpMode {

    DriveStates drive = new DriveStates("drive");
    ShooterState shooterState = new ShooterState("shooterStates");
    SequencerState sequencerState = new SequencerState("sequncer");
    IntakeState intakeState = new IntakeState("intake");
    AprilTagState aprilTagState = new AprilTagState("aprilTag");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        while (opModeIsActive()) {

            scheduler.schedule(new CommandDrive(drive, 1000));
            scheduler.run();
            scheduler.schedule(new CommandTurnRight(drive, 90));
            scheduler.run();
            scheduler.schedule(new CommandIntake(intakeState, 5000));
            scheduler.run();
            scheduler.schedule(new CommandTurnLeft(drive, 90));
            scheduler.run();
            scheduler.schedule(new CommandDrive(drive, -1000));
            scheduler.run();
            scheduler.schedule(new CommandSpinRight(sequencerState, 2000));
            scheduler.run();
            scheduler.schedule(new CommandShoot(shooterState, 4000, 5000));
            scheduler.run();


        }
    }
}
