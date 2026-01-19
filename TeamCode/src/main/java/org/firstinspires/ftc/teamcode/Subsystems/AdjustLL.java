

package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Tools.Constants.maxAngle;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


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
        return position * maxAngle;
    }

    public void setDegrees(double degrees) {
        double position = degrees / maxAngle;
        Adjust.setPosition(position);
    }
}