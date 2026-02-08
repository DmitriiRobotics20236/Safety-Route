package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Last_Mecanum;
import org.firstinspires.ftc.teamcode.modules.Servo_claw_plastina;
import org.firstinspires.ftc.teamcode.modules.Servo_flip;


@TeleOp(name = "MainTeleOp")
public class Mainclass extends LinearOpMode {


    // захваты пластин
    Servo servo_1_claw, servo_2_claw;

    private Servo_flip servoController_flip;

    Servo servo3, servo4;

    Last_Mecanum drive;


    @Override
    public void runOpMode() {
        drive = new Last_Mecanum(hardwareMap);


        servo_1_claw = hardwareMap.servo.get("servo1");
        servo_2_claw = hardwareMap.servo.get("servo2");

        Servo_claw_plastina servo1 = new Servo_claw_plastina(servo_1_claw, 0.04, 0.09); //левый серво
        Servo_claw_plastina servo2 = new Servo_claw_plastina(servo_2_claw, 0.06, 0.0); //правый серво


        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo4.setDirection(Servo.Direction.REVERSE);

        servoController_flip = new Servo_flip(servo3, servo4);


        waitForStart();

        while (opModeIsActive()) {
            servo1.update(gamepad2.left_bumper);
            servo2.update(gamepad2.right_bumper);


            if (gamepad2.left_trigger > 0.5) {
                servoController_flip.setPositionA();
            } else if (gamepad2.right_trigger > 0.5) {
                servoController_flip.setPositionB();
            }

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            drive.drive(y, x, rx);


        }
    }
}