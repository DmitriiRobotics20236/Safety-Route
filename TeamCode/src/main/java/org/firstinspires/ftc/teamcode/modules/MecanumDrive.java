package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrive {

    private DcMotor flMotor, frMotor, blMotor, brMotor;
    private IMU imu;

    private double targetHeading = 0;
    private boolean headingLocked = false;

    private double fl = 0, fr = 0, bl = 0, br = 0;
    private static final double SMOOTH = 0.20;
    private static final double HEADING_KP = 0.0;

    public MecanumDrive(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "left_up");
        frMotor = hardwareMap.get(DcMotor.class, "right_up");
        blMotor = hardwareMap.get(DcMotor.class, "left_down");
        brMotor = hardwareMap.get(DcMotor.class, "right_back");

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void drive(double x, double y, double turn, double speed) {

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double currentHeading = angles.getYaw(AngleUnit.RADIANS);

        double rotation;

        if (turn != 0) {
            rotation = turn;
            targetHeading = currentHeading;
            headingLocked = false;
        } else {
            if (!headingLocked) {
                targetHeading = currentHeading;
                headingLocked = true;
            }
            double error = targetHeading - currentHeading;
            error = Math.atan2(Math.sin(error), Math.cos(error));
            rotation = error * HEADING_KP;
        }

        double r = Math.hypot(x, y);
        double angle = Math.atan2(y, x) - Math.PI / 4;

        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        double max = Math.max(Math.abs(sin), Math.abs(cos));
        if (max < 1e-6) max = 1;

        double flTarget = (r * cos / max + rotation) * speed;
        double frTarget = (r * sin / max - rotation) * speed;
        double blTarget = (r * sin / max + rotation) * speed;
        double brTarget = (r * cos / max - rotation) * speed;

        double maxPower = Math.max(
                Math.max(Math.abs(flTarget), Math.abs(frTarget)),
                Math.max(Math.abs(blTarget), Math.abs(brTarget))
        );

        if (maxPower > 1.0) {
            flTarget /= maxPower;
            frTarget /= maxPower;
            blTarget /= maxPower;
            brTarget /= maxPower;
        }

        fl += (flTarget - fl) * SMOOTH;
        fr += (frTarget - fr) * SMOOTH;
        bl += (blTarget - bl) * SMOOTH;
        br += (brTarget - br) * SMOOTH;

        flMotor.setPower(fl);
        frMotor.setPower(fr);
        blMotor.setPower(bl);
        brMotor.setPower(br);
    }

    public double getHeading() {
        return Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}