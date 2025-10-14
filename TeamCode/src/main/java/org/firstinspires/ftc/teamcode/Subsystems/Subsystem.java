
package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Commands.Command;

public abstract class Subsystem {
    private String name;

    public Subsystem(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
    public void periodic(){}
}
