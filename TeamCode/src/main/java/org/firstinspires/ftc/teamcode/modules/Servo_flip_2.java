package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.Servo;

public class Servo_flip_2 {

    private Servo servo_left_front;
    private Servo servo_right_front;

    // Два фиксированных положения
    private static final double POSITION_A = 0;
    private static final double POSITION_B = 0.5;

    public Servo_flip_2(Servo servo1, Servo servo2) {
        this.servo_left_front = servo1;
        this.servo_right_front = servo2;

        setPositionA_2(); // стартовое положение
    }

    public void setPositionA_2() {
        servo_left_front.setPosition(POSITION_A);
        servo_right_front.setPosition(POSITION_A);
    }

    public void setPositionB_2() {
        servo_left_front.setPosition(POSITION_B);
        servo_right_front.setPosition(POSITION_B);
    }
}
