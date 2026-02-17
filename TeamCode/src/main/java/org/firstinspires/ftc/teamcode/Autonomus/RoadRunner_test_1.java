package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Actions.ArmAction_RR;
import org.firstinspires.ftc.teamcode.Actions.PlastinaAction;
import org.firstinspires.ftc.teamcode.Actions.ServoFlipAction;
import org.firstinspires.ftc.teamcode.OpenCV.TET;
import org.firstinspires.ftc.teamcode.RoadRunner_classes.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Servo_claw_plastina;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "first_auto", group = "AUTO")
public class RoadRunner_test_1 extends LinearOpMode {

    OpenCvCamera camera;
    TET pipeline;
    Servo servo3, servo4; // серво переворот

    Servo servo1, servo2; // серво пластина


    @Override
    public void runOpMode() {
        //рука задняя
        ArmAction_RR arm = new ArmAction_RR(hardwareMap);
        // СервоПластина
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo_claw_plastina claw1 = new Servo_claw_plastina(servo1, 0.04, 0.09); //левый
        Servo_claw_plastina claw2 = new Servo_claw_plastina(servo2, 0.06, 0.0); //правый

        //СервоПереворот
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo4.setDirection(Servo.Direction.REVERSE);
        double POS_A = -1.0;
        double POS_B = 0.45;
        //Езда
        Pose2d beginPose = new Pose2d(new Vector2d(-70, -12), Math.toRadians(0));
        MecanumDrive driveMEC = new MecanumDrive(hardwareMap, beginPose);
        //камера
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        pipeline = new TET();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (opModeInInit()) {
            //сюда хватить пластин

        }

        waitForStart();

        Action First_path = driveMEC.actionBuilder(beginPose)
                .lineToX(-34)
                .strafeTo(new Vector2d(-34, -17))
                .stopAndAdd((new ServoFlipAction(servo3, servo4, POS_A, POS_B, ServoFlipAction.TargetPosition.A)))
                .stopAndAdd(new PlastinaAction(claw1, true))
                .stopAndAdd(new PlastinaAction(claw2, true))
                .stopAndAdd(arm.createAction(300, 0.6, telemetry))
                .stopAndAdd(new PlastinaAction(claw1, false))
                .stopAndAdd(new PlastinaAction(claw2, false))
                .stopAndAdd((new ServoFlipAction(servo3, servo4, POS_A, POS_B, ServoFlipAction.TargetPosition.B)))
                .stopAndAdd(arm.createAction(50, 0.6, telemetry))
                .turnTo(Math.toRadians(0))
                .build();
        Actions.runBlocking(new SequentialAction(First_path));

        camera.startStreaming(640, 480);

        if (pipeline.signDetected) { //если в правой зоне
            camera.stopStreaming();
            Action Second_path = driveMEC.actionBuilder(driveMEC.localizer.getPose())
                    .build();
            Actions.runBlocking(new SequentialAction(Second_path));
        } else {
            Action Third_path = driveMEC.actionBuilder(driveMEC.localizer.getPose())
                    .strafeTo(new Vector2d(-34, 3))
                    .build();
            Actions.runBlocking(new SequentialAction(Third_path));
            if (pipeline.signDetected) { //если по центру
                camera.stopStreaming();
                Action Forth_path = driveMEC.actionBuilder(driveMEC.localizer.getPose())
                        .build();
                Actions.runBlocking(new SequentialAction(Forth_path));
            } else { // если нет ни там, ни там, то едем в самую левую зону
                camera.stopStreaming();
                Action Fifth_path = driveMEC.actionBuilder(driveMEC.localizer.getPose())
                        .strafeTo(new Vector2d(-34, 15))

                        .build();
                Actions.runBlocking(new SequentialAction(Fifth_path));

            }
        }

    }
}



//.stopAndAdd((new ServoFlipAction(servo3, servo4, POS_A, POS_B,
//            ServoFlipAction.TargetPosition.A)))
//        .splineTo(new Vector2d(45,45), Math.PI)
//        .stopAndAdd(new PlastinaAction(claw1,true))
//        .stopAndAdd(new PlastinaAction(claw2,false))
//        .build();
