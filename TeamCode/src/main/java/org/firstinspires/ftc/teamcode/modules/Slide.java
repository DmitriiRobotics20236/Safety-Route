package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slide {

    private DcMotor slideLeft;
    private DcMotor slideRight;

    // Очень мягкий PID
    private double kP = 0.002;
    private double kD = 0.0003;

    private double lastError = 0;
    private double currentPower = 0;

    private int targetPosition = 0;
    private boolean pidEnabled = false;

    private ElapsedTime timer = new ElapsedTime();

    // Жёсткие ограничения для экономии батареи
    private final double MAX_POWER = 0.9;
    private final int TOLERANCE = 25;
    private final double RAMP_SPEED = 0.02;

    public Slide(HardwareMap hardwareMap) {

        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Переключение PID по кнопке A
    public void togglePID() {

        pidEnabled = !pidEnabled;

        if (pidEnabled) {
            targetPosition = slideLeft.getCurrentPosition();
            lastError = 0;
            timer.reset();
        } else {
            setPowerSmooth(0); // полностью выключаем
        }
    }

    public void update(double stickInput) {

        // Если PID выключен → только manual
        if (!pidEnabled) {
            manual(stickInput);
            return;
        }

        updatePID();
    }

    private void updatePID() {

        int currentPosition = slideLeft.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Если в зоне допуска — полностью выключаем моторы
        if (Math.abs(error) < TOLERANCE) {
            setPowerSmooth(0);
            return;
        }

        double dt = timer.seconds();
        timer.reset();

        double derivative = (error - lastError) / dt;

        double output = (kP * error) + (kD * derivative);
        output = Range.clip(output, -MAX_POWER, MAX_POWER);

        setPowerSmooth(output);

        lastError = error;
    }

    private void manual(double power) {
        power = Range.clip(power, -0.9, 0.9);
        setPowerSmooth(power);
    }

    // Плавная подача мощности (убирает пики тока)
    private void setPowerSmooth(double targetPower) {

        if (currentPower < targetPower) {
            currentPower += RAMP_SPEED;
        } else if (currentPower > targetPower) {
            currentPower -= RAMP_SPEED;
        }

        currentPower = Range.clip(currentPower, -MAX_POWER, MAX_POWER);

        slideLeft.setPower(currentPower);
        slideRight.setPower(currentPower);
    }

    // Телеметрия
    public int getLeftTicks() { return slideLeft.getCurrentPosition(); }
    public int getRightTicks() { return slideRight.getCurrentPosition(); }
    public boolean isPidEnabled() { return pidEnabled; }
    public double getPower() { return currentPower; }
}