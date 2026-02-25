package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ToggleServo {

    private Servo servo;

    // Положения сервопривода
    private final double POSITION_ONE = 0.25;
    private final double POSITION_TWO = 0.52;

    private boolean state = false; // false = POSITION_ONE, true = POSITION_TWO

    public ToggleServo(HardwareMap hardwareMap, String servoName) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo.setPosition(POSITION_ONE);
    }

    public void toggle() {
        state = !state;

        if (state) {
            servo.setPosition(POSITION_TWO);
        } else {
            servo.setPosition(POSITION_ONE);
        }
    }

    public boolean getState() {
        return state;
    }
}