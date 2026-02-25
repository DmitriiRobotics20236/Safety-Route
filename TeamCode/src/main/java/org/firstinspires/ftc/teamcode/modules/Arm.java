package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Arm {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public Arm(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor_arm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor_arm");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    // ===== Ручное управление =====
    public void manualControl(double stickValue) {

        double power = stickValue;
        power = Range.clip(power, -0.65, 0.65);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}



