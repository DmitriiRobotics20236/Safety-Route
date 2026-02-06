package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.hardware.Servo;

public class Servo_claw_plastina {

    private final Servo servo;
    private final double openPos;
    private final double closePos;

    private boolean isOpen = false;
    private boolean lastButtonState = false;

    public Servo_claw_plastina(Servo servo, double openPos, double closePos) {
        this.servo = servo;
        this.openPos = openPos;
        this.closePos = closePos;
        servo.setPosition(closePos);
    }

    public void update(boolean buttonPressed) {
        if (buttonPressed && !lastButtonState) {
            isOpen = !isOpen;
            servo.setPosition(isOpen ? openPos : closePos);
        }
        lastButtonState = buttonPressed;
    }
}

