package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Drive.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.Intake.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Shooter.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.Queuer.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnRight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Queuer.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterState;
import org.json.JSONException;

@Autonomous
public class averyAuto extends LinearOpMode {

    DriveStates drive = new DriveStates("drive");
    ShooterState shooterState = new ShooterState("shooterStates");
    SequencerState sequencerState = new SequencerState("sequncer");
    IntakeState intakeState = new IntakeState("intake");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        while (opModeIsActive()) {

            scheduler.schedule(new CommandDrive(drive, 1000));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            scheduler.schedule(new CommandTurnRight(drive, 90));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            scheduler.schedule(new CommandIntake(intakeState, 5000));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            scheduler.schedule(new CommandTurnLeft(drive, 90));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            scheduler.schedule(new CommandDrive(drive, -1000));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            scheduler.schedule(new CommandSpinRight(sequencerState, 2000));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            scheduler.schedule(new CommandShoot(shooterState, 4000, 5000));
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }


        }
    }
}
