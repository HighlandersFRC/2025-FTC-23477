package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Queuer.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.Queuer.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterState;


public class NewRobot {

    // Instance variables for subsystems
    public Drive drive;
    public ShooterState shooterStates;
    public IntakeState intakeStates;
    public SequencerState sequencerState;
    public DriveStates driveStates;
    public IndexerState indexerState;
    public QueueState queueState;
    public NewRobot(HardwareMap hardwareMap) {
        this.shooterStates = new ShooterState("shooter");
        this.intakeStates = new IntakeState("intake");
        this.sequencerState = new SequencerState("sequencer");
        this.driveStates =  new DriveStates("drive");
        this.indexerState = new IndexerState("index");
        this.queueState = new QueueState("queue");
    }

    public void run() {

    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap) {
        shooterStates.init(hardwareMap);
        intakeStates.init(hardwareMap);
        sequencerState.init(hardwareMap);
        driveStates.init(hardwareMap);
        indexerState.init(hardwareMap);
        queueState.init(hardwareMap);
    }

}
