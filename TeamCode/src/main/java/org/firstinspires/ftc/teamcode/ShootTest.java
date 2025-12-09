package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

@TeleOp
public class ShootTest extends LinearOpMode {

ShooterState shooterState = new ShooterState("shoot");
SequencerState sequencerState = new SequencerState("sequence");
IntakeState intakeState = new IntakeState("intkae");
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);
        intakeState.init(hardwareMap);
        Drive drive = new Drive("drive", hardwareMap);
        while (opModeIsActive()) {
            sequencerState.periodic();
            shooterState.periodic();
            intakeState.init(hardwareMap);
            if (gamepad1.right_bumper) {
                shooterState.setTargetRPM(5600);
                shooterState.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
            } else if (gamepad1.b) {
                sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.SPIN_RIGHT);
            } else if (gamepad1.left_bumper) {
                intakeState.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
            }else {
                sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.DEFAULT);
                shooterState.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
                intakeState.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
            }


            drive.FeildCentric(gamepad1);
        }
    }
}
