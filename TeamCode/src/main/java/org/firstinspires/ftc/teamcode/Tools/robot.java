package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.rotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;


public class robot {

    // Instance variables for subsystems
    public rotateSubsystem rotate;
    public shooterSubsystem shooter;

    public robot(HardwareMap hardwareMap) {

        this.rotate = new rotateSubsystem("rotate");
        this.shooter = new shooterSubsystem("shooter");
    }

    public void run() {

    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap) {

        rotate.init(hardwareMap);
        shooter.init(hardwareMap);
    }

}