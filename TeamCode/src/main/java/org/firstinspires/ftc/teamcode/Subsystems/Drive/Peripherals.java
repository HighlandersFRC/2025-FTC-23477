package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Peripherals extends Subsystem {
    private double fieldX;
    private double fieldY;
    private double theta;
    SparkFunOTOS mouse;

    public Peripherals(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {

        mouse = hardwareMap.get(SparkFunOTOS.class, Constants.InitInfo.Drive.MOUSE_NAME);
        mouse.setLinearUnit(DistanceUnit.METER);
        mouse.setAngularUnit(AngleUnit.DEGREES);
        mouse.setOffset(Constants.Physical.MOUSE_SENSOR_MOUNTED_POSE);
        mouse.setLinearScalar(Constants.Physical.MOUSE_SENSOR_LINEAR_SCALAR);
        mouse.setAngularScalar(Constants.Physical.MOUSE_SENSOR_ANGULAR_SCALAR);
        configureMouseSensor();
    }

    public void configureMouseSensor() {
    mouse.calibrateImu();
    mouse.resetTracking();
    }

    public void resetOdometry(Pose2D newPose) {
        mouse.setPosition(new SparkFunOTOS.Pose2D(newPose.getX(DistanceUnit.METER), newPose.getY(DistanceUnit.METER), newPose.getHeading(AngleUnit.DEGREES)));
    }

    public void updateMouse() {
        SparkFunOTOS.Pose2D field = mouse.getPosition();
        fieldX = field.x;
        fieldY = field.y;
        theta = field.h;
    }

    public double getX() {
        return fieldX;
    }

    public double getY() {
        return fieldY;
    }

    public double getYaw() {
        return theta;
    }

}
