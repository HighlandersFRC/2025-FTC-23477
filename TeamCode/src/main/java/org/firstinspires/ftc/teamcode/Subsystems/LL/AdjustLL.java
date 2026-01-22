

package org.firstinspires.ftc.teamcode.Subsystems.LL;


import static org.firstinspires.ftc.teamcode.Tools.Constants.MAX_ANGLE_2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;


public class AdjustLL extends Subsystem {

    Servo Adjust;



    public AdjustLL(String name) {
        super(name);
    }


    public void init(HardwareMap hardwareMap) {
        Adjust = hardwareMap.get(Servo.class, "AdjustLL");
    }

    public double getDegrees() {
        double position = Adjust.getPosition();
        return position * MAX_ANGLE_2;
    }

    public void setDegrees(double degrees) {
        double position = degrees / MAX_ANGLE_2;
        Adjust.setPosition(position);
    }
}