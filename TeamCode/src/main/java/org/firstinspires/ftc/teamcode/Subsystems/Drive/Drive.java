package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class Drive extends Subsystem {
    Peripherals peripherals;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    public Drive(String name, Peripherals peripherals) {
        super(name);
        this.peripherals = peripherals;
    }

    public void init(HardwareMap hardwareMap) {
        peripherals.init(hardwareMap);
        frontRightMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.FRONT_RIGHT_NAME);
        frontLeftMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.FRONT_LEFT_NAME);
        backRightMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.BACK_RIGHT_NAME);
        backLeftMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.BACK_LEFT_NAME);
    }

    public void setMotors(Vector vector, double turnValue) {
        double x = vector.getI();
        double y = vector.getJ();

        double frontLeft  = y + x + turnValue;
        double frontRight = y - x - turnValue;
        double backLeft   = y - x + turnValue;
        double backRight  = y + x - turnValue;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeft),
                        Math.max(
                                Math.abs(frontRight),
                                Math.max(Math.abs(backLeft), Math.abs(backRight))
                        )
                )
        );

        frontLeft  /= max;
        frontRight /= max;
        backLeft   /= max;
        backRight  /= max;

        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }
    public void teleopFieldCentric(double oiRX, double oiLX, double oiLY) {
        peripherals.updateMouse();

        Vector translation = new Vector(
                oiLX * Constants.Physical.TOP_SPEED,
                -oiLY * Constants.Physical.TOP_SPEED);
        double rotation = oiRX * Constants.Physical.MAX_ANGULAR_SPEED;

        double yawRadians = Math.toRadians(peripherals.getYaw());

        double cos = Math.cos(yawRadians);
        double sin = Math.sin(yawRadians);

        double x = translation.getI() * cos - translation.getJ() * sin;
        double y = translation.getI() * sin + translation.getJ() * cos;

        Vector robotVector = new Vector(x, y);

        setMotors(robotVector, rotation);

    }


}
