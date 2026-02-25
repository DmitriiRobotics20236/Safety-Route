package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Actions.ArmAction_RR;
import org.firstinspires.ftc.teamcode.Actions.ServoFlipAction;
import org.firstinspires.ftc.teamcode.OpenCV.TET;
import org.firstinspires.ftc.teamcode.RoadRunner_classes.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "first_auto", group = "AUTO")
@Config
public class RoadRunner_test_1 extends LinearOpMode {

    // ===== DASHBOARD ПЕРЕМЕННЫЕ =====

    // Стартовая поза
    public static double START_X = -70;
    public static double START_Y = -12;
    public static double START_HEADING = 0;

    // Точки движения
    public static double P1_X = -41;
    public static double P1_Y = -12;

    public static double P2_X = -41;
    public static double P2_Y = -5;

    public static double P3_X = -45;
    public static double P3_Y = -4;

    public static double TURN_ANGLE = 270;

    public static double P4_X = -40;
    public static double P4_Y = -52;

    public static double P5_X = 12;
    public static double P5_Y = -52;

    // Серво
    public static double HOLD_POSITION = 0.52;
    public static double HOLD_TOP = 0.0;

    public static double POS_A = -1.0;
    public static double POS_B = 0.45;

    // Таймеры
    public static double DETECT_TIME = 3.0;
    public static double WAIT_TIME = 1.0;

    // ==================================

    OpenCvCamera camera;
    TET pipeline;

    Servo serv5, serv6;
    Servo servo3, servo4;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ArmAction_RR arm = new ArmAction_RR(hardwareMap);

        serv5 = hardwareMap.get(Servo.class, "left_2");
        serv6 = hardwareMap.get(Servo.class, "right_2");
        serv6.setDirection(Servo.Direction.REVERSE);

        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo4.setDirection(Servo.Direction.REVERSE);

        // ===== ROADRUNNER =====
        Pose2d beginPose = new Pose2d(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING)
        );

        MecanumDrive driveMEC = new MecanumDrive(hardwareMap, beginPose);

        // ===== CAMERA =====
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
            public void onError(int errorCode) {}
        });

        while (opModeInInit()) {
        }

        waitForStart();
        if (isStopRequested()) return;
        boolean found = false;
        ElapsedTime detectTimer = new ElapsedTime();

        while (opModeIsActive() && detectTimer.seconds() < DETECT_TIME) {

            serv5.setPosition(HOLD_TOP);
            serv6.setPosition(HOLD_TOP);

            if (pipeline.signDetected) {
                found = true;
            }
            telemetry.addData("Detecting", pipeline.signDetected);
            telemetry.update();
        }
        camera.stopStreaming();
        camera.closeCameraDevice();

        if (found) {
            camera.stopStreaming();
            camera.closeCameraDevice();

            serv5.setPosition(HOLD_TOP);
            serv6.setPosition(HOLD_TOP);

            Action movePath = driveMEC.actionBuilder(beginPose)
                    .strafeToLinearHeading(new Vector2d(P1_X, P1_Y), 0)
                    .strafeToLinearHeading(new Vector2d(P2_X, P2_Y), 0)
                    .strafeToLinearHeading(new Vector2d(P3_X, P3_Y), 0)
                    .waitSeconds(WAIT_TIME)
                    .strafeToLinearHeading(new Vector2d(P4_X, P4_Y), Math.toRadians(TURN_ANGLE))
                    .strafeToLinearHeading(new Vector2d(P5_X, P5_Y), Math.toRadians(TURN_ANGLE))
                    .stopAndAdd(arm.createTimedAction(1.3, -0.65, telemetry))
                    .stopAndAdd(new ServoFlipAction(servo3, servo4, POS_A, POS_B,
                            ServoFlipAction.TargetPosition.A))
                    .build();
            Actions.runBlocking(new SequentialAction(movePath));
        }

        // ===== ЕСЛИ НЕ НАЙДЕН =====
        else {
            while (opModeIsActive()) {
                serv5.setPosition(HOLD_POSITION);
                serv6.setPosition(HOLD_POSITION);
                telemetry.addLine("NO SIGN - WAITING");
                telemetry.update();
            }
        }
    }
}