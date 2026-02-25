package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.Servo;

public class Servo_flip {

    private Servo servo1_up;
    private Servo servo2_up;

    // Два фиксированных положения
    private static final double POSITION_A = 0.035 ;
    private static final double POSITION_B = 0.35;

    public Servo_flip(Servo servo1, Servo servo2) {
        this.servo1_up = servo1;
        this.servo2_up = servo2;

        setPositionA(); // стартовое положение
    }

    public void setPositionA() {
        servo1_up.setPosition(POSITION_A);
        servo2_up.setPosition(POSITION_A);
    }

    public void setPositionB() {
        servo1_up.setPosition(POSITION_B);
        servo2_up.setPosition(POSITION_B);
    }
}
