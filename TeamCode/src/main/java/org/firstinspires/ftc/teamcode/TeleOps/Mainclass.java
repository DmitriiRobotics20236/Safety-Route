package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.Last_Mecanum;
import org.firstinspires.ftc.teamcode.modules.Servo_claw_plastina;
import org.firstinspires.ftc.teamcode.modules.Servo_flip;
import org.firstinspires.ftc.teamcode.modules.Servo_flip_2;
import org.firstinspires.ftc.teamcode.modules.Slide;
import org.firstinspires.ftc.teamcode.modules.ToggleServo;


@TeleOp(name = "MainTeleOp")
public class Mainclass extends LinearOpMode {
    // захваты пластин
    Servo servo_1_claw, servo_2_claw;
    //Захват переворот зад
    private Servo_flip servoController_flip;
    boolean flipState = false;
    boolean lastTriggerState = false;
    Servo servo3, servo4;
    // Перемещение
    Last_Mecanum drive;

    private Arm arm;
    private Slide slide;
    boolean lastA = false;

    private Servo_flip_2 servCont_flip2;
    boolean flipState2 = false;
    boolean lastTriggerflip_2 = false;
    Servo left_2, right_2;


    private ToggleServo toggleServo;

    private boolean lastDpadState = false;


    @Override
    public void runOpMode() {
        drive = new Last_Mecanum(hardwareMap);

        servo_1_claw = hardwareMap.servo.get("servo1");
        servo_2_claw = hardwareMap.servo.get("servo2");

        Servo_claw_plastina servo1 = new Servo_claw_plastina(servo_1_claw, 0.035, 0.09); //левый серво
        Servo_claw_plastina servo2 = new Servo_claw_plastina(servo_2_claw, 0.045, 0.0); //правый серво

        toggleServo = new ToggleServo(hardwareMap, "pedestrian");

        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo4.setDirection(Servo.Direction.REVERSE);
        servoController_flip = new Servo_flip(servo3, servo4);

        left_2 = hardwareMap.get(Servo.class, "left_2");
        right_2 = hardwareMap.get(Servo.class, "right_2");
        right_2.setDirection(Servo.Direction.REVERSE);
        servCont_flip2 = new Servo_flip_2(left_2, right_2);



        arm = new Arm(hardwareMap);
        slide = new Slide(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            servo1.update(gamepad2.left_bumper);
            servo2.update(gamepad2.right_bumper);

            boolean currentDpadState = gamepad2.dpad_up;
            if (currentDpadState && !lastDpadState) {
                toggleServo.toggle();
            }

            lastDpadState = currentDpadState;
            telemetry.addData("Servo State", toggleServo.getState() ? "POSITION_TWO" : "POSITION_ONE");
            telemetry.update();


            boolean currentTriggerState = gamepad2.right_trigger > 0.5;
            if (currentTriggerState && !lastTriggerState) {
                flipState = !flipState;

                if (flipState) {
                    servoController_flip.setPositionB();
                } else {
                    servoController_flip.setPositionA();
                }
            }

            lastTriggerState = currentTriggerState;
            //........................................................................
            boolean CurrenTrigger_left = gamepad2.left_trigger > 0.5;
            if (CurrenTrigger_left && !lastTriggerflip_2) {
                flipState2 = !flipState2;

                if (flipState2) {
                    servCont_flip2.setPositionB_2();
                } else {
                    servCont_flip2.setPositionA_2();
                }
            }
            lastTriggerflip_2 = CurrenTrigger_left;


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double leftTrigger = gamepad1.left_trigger;
            drive.drive(y, x, rx, leftTrigger);



            // ===== СТИК =====
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                arm.manualControl(gamepad2.left_stick_y);
            }


            boolean currentA = gamepad2.a;

            if (currentA && !lastA) {
                slide.togglePID();
            }

            lastA = currentA;

            double stick = gamepad2.right_stick_y;

            slide.update(stick);

            telemetry.addLine("===== SLIDE =====");
            telemetry.addData("Left ticks", slide.getLeftTicks());
            telemetry.addData("Right ticks", slide.getRightTicks());
            telemetry.addData("PID Enabled", slide.isPidEnabled());
            telemetry.addData("Power", slide.getPower());
            telemetry.update();
        }
    }
}
