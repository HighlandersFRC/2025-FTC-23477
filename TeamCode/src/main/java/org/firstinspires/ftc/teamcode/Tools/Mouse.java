package org.firstinspires.ftc.teamcode.Tools;

import static org.firstinspires.ftc.teamcode.Tools.Constants.ANGULAR_SCALER;
import static org.firstinspires.ftc.teamcode.Tools.Constants.LINEAR_SCALER;
import static org.firstinspires.ftc.teamcode.Tools.Constants.offset;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class Mouse {
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;

    public static void init(HardwareMap hardwareMap) {
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
    }
    public static void configureOtos() {
        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        mouse.setOffset(offset);
        mouse.setLinearScalar(LINEAR_SCALER);
        mouse.setAngularScalar(ANGULAR_SCALER);
        mouse.calibrateImu();
        mouse.resetTracking();
        System.out.println("configured");
        System.out.println("theta"+offset.h);
    }
    public static void setPosition(double x, double y, double theta){
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, theta);
        mouse.setPosition(currentPosition);
    }
    public static void update() {
        field = mouse.getPosition();
        fieldX = field.x;
        fieldY = field.y;
        theta = field.h;
    }
    public static double getX() {
        return -fieldX;
    }
    public static double getY() {
        return fieldY;
    }
    public static double getTheta() {
        return theta;
    }


}