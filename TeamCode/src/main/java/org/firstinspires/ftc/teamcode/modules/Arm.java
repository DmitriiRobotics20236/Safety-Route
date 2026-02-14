package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Arm {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Позиции в тиках
    public int POS_LOW = 500;
    public int POS_MID = 1200;
    public int POS_HIGH = 2000;
    public int POS_MAX = 3000;

    public Arm(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor_arm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor_arm");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===== Ручное управление =====
    public void manualControl(double stickValue) {

        double power = -stickValue;
        power = Range.clip(power, -0.8, 0.8);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    // ===== Движение в позицию =====
    public void moveToPosition(int target) {

        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.8);
        rightMotor.setPower(0.8);
    }

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition();
    }
}
