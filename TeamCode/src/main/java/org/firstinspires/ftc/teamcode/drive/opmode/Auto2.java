package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RingCounter;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(group = "drive")
public class Auto2 extends LinearOpMode {

    private FtcDashboard dashboard;

    public int rings;

    public static int middleY = -48;
    public static int leftY = -36;
    public static int rightY = -60;

    public static int startY = middleY;

    public static int backUpY = leftY;
    public static int dodgeWallY = leftY;
    public static int shoot1Y = leftY;
    public static int suck1Y = leftY;
    public static int suck2Y = leftY;
    public static int shoot2Y = leftY;
    public static int parkY = leftY;

    public static int swerveY = rightY;
    public static int zoneAY = rightY;
    public static int zoneCY = rightY;

    public static int zoneBY = -36;

    public static int startX = -60;
    public static int swerveX = -24;
    public static int zoneAX = 0;
    public static int zoneBX = 24;
    public static int zoneCX = 42;
    public static int backUpX = -6;
    public static int dodgeWallX = 0;
    public static int shoot1X = -12;
    public static int suck1X = -24;
    public static int suck2X = -36;
    public static int shoot2X = -12;
    public static int parkX = 12;

    public static Vector2d startV = new Vector2d(startX, startY);
    public static Pose2d startP = new Pose2d(startX, startY, Math.toRadians(180));

    public static Vector2d swerveV = new Vector2d(swerveX, swerveY);
    public static Pose2d swerveP = new Pose2d(swerveX, swerveY, Math.toRadians(180));

    public static Vector2d zoneAV = new Vector2d(zoneAX, zoneAY);
    public static Pose2d zoneAP = new Pose2d(zoneAX, zoneAY, Math.toRadians(180));

    public static Vector2d zoneBV = new Vector2d(zoneBX, zoneBY);
    public static Pose2d zoneBP = new Pose2d(zoneBX, zoneCY, Math.toRadians(180));

    public static Vector2d zoneCV = new Vector2d(zoneCX, zoneCY);
    public static Pose2d zoneCP = new Pose2d(zoneCX, zoneCY, Math.toRadians(180));

    public static Vector2d backUpV = new Vector2d(backUpX, backUpY);
    public static Pose2d backUpP = new Pose2d(backUpX, backUpY, Math.toRadians(180));

    public static Vector2d dodgeWallV = new Vector2d(dodgeWallX, dodgeWallY);
    public static Pose2d dodgeWallP = new Pose2d(dodgeWallX, dodgeWallY, Math.toRadians(180));

    public static Vector2d shoot1V = new Vector2d(shoot1X, shoot1Y);
    public static Pose2d shoot1P = new Pose2d(shoot1X, shoot1Y, Math.toRadians(0));
    public static Pose2d shoot1PA = new Pose2d(shoot1X, shoot1Y, Math.toRadians(5));
    public static Pose2d shoot1PB = new Pose2d(shoot1X + 12, shoot1Y, Math.toRadians(5));

    public static Vector2d suck1V = new Vector2d(suck1X, suck1Y);
    public static Pose2d suck1P = new Pose2d(suck1X, suck1Y, Math.toRadians(180));
    public static Pose2d suck1PB = new Pose2d(suck1X + 12, suck1Y, Math.toRadians(180));

    public static Vector2d suck2V = new Vector2d(suck2X, suck2Y);
    public static Pose2d suck2P = new Pose2d(suck2X, suck2Y, Math.toRadians(180));

    public static Vector2d shoot2V = new Vector2d(shoot2X, shoot2Y);
    public static Pose2d shoot2P = new Pose2d(shoot2X, shoot2Y, Math.toRadians(0));

    public static Vector2d parkV = new Vector2d(parkX, parkY);
    public static Pose2d parkP = new Pose2d(parkX, parkY, Math.toRadians(0));

    public static double shooterAngle = 25;
    public static double vel = 1700;

    public static int shootCount = 3;
    public static int shootWait = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot roboto = new Robot(hardwareMap);

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


        Trajectory startToSwerve = drive.trajectoryBuilder(startP) // A, B, C
                .lineToConstantHeading(swerveV)
                .build();

        Trajectory swerveToZoneA = drive.trajectoryBuilder(startToSwerve.end()) // A
                .splineToConstantHeading(zoneAV, 0)
                .build();

        /* Trajectory zoneAToDodgeWall = drive.trajectoryBuilder(swerveToZoneA.end()) // A
                .splineToSplineHeading(dodgeWallP, 180)
                .build();
         */

        /* Trajectory zoneAToDodgeWall = drive.trajectoryBuilder(swerveToZoneA.end()) // A
                .strafeTo(dodgeWallV)
                .build();
         */

        Trajectory zoneAToBackUp = drive.trajectoryBuilder(swerveToZoneA.end()) // A
                .splineToConstantHeading(backUpV, 0)
                .build();

        Trajectory backUpToDodgeWall = drive.trajectoryBuilder(swerveToZoneA.end()) // A
                .splineToConstantHeading(dodgeWallV, 0)
                .build();

        Trajectory dodgeWallToShoot1 = drive.trajectoryBuilder(zoneAToBackUp.end()) // A
                .splineToSplineHeading(shoot1PA, 0)
                .build();

        Trajectory shoot1ToPark = drive.trajectoryBuilder(dodgeWallToShoot1.end()) // A
                .splineToConstantHeading(parkV, 0)
                .build();

        Trajectory swerveToZoneB = drive.trajectoryBuilder(startToSwerve.end()) // B
                .splineToConstantHeading(zoneBV, 90)
                .build();

        Trajectory zoneBToShoot1 = drive.trajectoryBuilder(swerveToZoneB.end()) // B
                .splineToSplineHeading(shoot1PB, 0)
                .build();

        Trajectory shoot1ToSuck1B = drive.trajectoryBuilder(zoneBToShoot1.end()) // B
                .splineToSplineHeading(suck1PB, 0)
                .build();

        Trajectory suck1ToSuck2B = drive.trajectoryBuilder(shoot1ToSuck1B.end()) // B
                .splineToConstantHeading(suck2V, 0)
                .build();

        Trajectory suck2ToShoot2B = drive.trajectoryBuilder(suck1ToSuck2B.end()) // B
                .splineToSplineHeading(shoot2P, 0)
                .build();

        Trajectory shoot2ToParkB = drive.trajectoryBuilder(suck2ToShoot2B.end()) // B
                .splineToConstantHeading(parkV, 0)
                .build();

        Trajectory swerveToZoneC = drive.trajectoryBuilder(startToSwerve.end()) // C
                .splineToConstantHeading(zoneCV, 0)
                .build();

        Trajectory zoneCToShoot1 = drive.trajectoryBuilder(swerveToZoneC.end()) // C
                .splineToSplineHeading(shoot1P, 180)
                .build();

        Trajectory shoot1ToSuck1C = drive.trajectoryBuilder(zoneCToShoot1.end()) // C
                .splineToSplineHeading(suck1P, 0)
                .build();

        Trajectory suck1ToSuck2C = drive.trajectoryBuilder(shoot1ToSuck1C.end()) // C
                .splineToConstantHeading(suck2V, 0)
                .build();

        Trajectory suck2ToShoot2C = drive.trajectoryBuilder(suck1ToSuck2C.end()) // C
                .splineToSplineHeading(shoot2P, 0)
                .build();

        Trajectory shoot2ToParkC = drive.trajectoryBuilder(suck2ToShoot2C.end()) // C
                .splineToConstantHeading(parkV, 0)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName()
        );
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        RingCounter Processing = new RingCounter();
        camera.setPipeline(Processing);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        camera.closeCameraDevice();

        waitForStart();

        rings = Processing.getRingCount();

        telemetry.addData("Rings", rings);
        telemetry.update();

        if (rings == 0) { // Zone A
            drive.followTrajectory(startToSwerve);
            drive.followTrajectory(swerveToZoneA);

            roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
            sleep(500);
            roboto.setWobbleGrabberMode(false);
            sleep(500);

            drive.followTrajectory(zoneAToBackUp);
            drive.followTrajectory(backUpToDodgeWall);
            drive.followTrajectory(dodgeWallToShoot1);

            // shoot 3x + 3
            roboto.autoStartShoot(shooterAngle, vel);
            sleep(1100);
            roboto.setRingBlockerMode(false);
            for (int i = 0; i < shootCount + 3; i++) {
                roboto.setRingPusherMode(true);
                sleep(shootWait);
                roboto.setRingPusherMode(false);
                sleep(shootWait);
            }
            roboto.autoStopShoot();

            drive.followTrajectory(shoot1ToPark);

            // roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
            sleep(1500);
        }
        else if (rings == 1) { // Zone B
            drive.followTrajectory(startToSwerve);
            drive.followTrajectory(swerveToZoneB);

            roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
            sleep(500);
            roboto.setWobbleGrabberMode(false);
            sleep(500);

            drive.followTrajectory(zoneBToShoot1);

            // shoot 3x + 3
            roboto.autoStartShoot(shooterAngle, vel);
            sleep(1100);
            roboto.setRingBlockerMode(false);
            for (int i = 0; i < shootCount + 3; i++) {
                roboto.setRingPusherMode(true);
                sleep(shootWait);
                roboto.setRingPusherMode(false);
                sleep(shootWait);
            }
            roboto.autoStopShoot();

            drive.followTrajectory(shoot1ToSuck1B);

            roboto.setIntakeMode(true);

            sleep(100);

            drive.followTrajectory(suck1ToSuck2B);

            sleep(100);

            roboto.setIntakeMode(false);

            drive.followTrajectory(suck2ToShoot2B);

            // shoot 3x
            roboto.autoStartShoot(shooterAngle, vel);
            sleep(1100);
            roboto.setRingBlockerMode(false);
            for (int i = 0; i < shootCount; i++) {
                roboto.setRingPusherMode(true);
                sleep(shootWait);
                roboto.setRingPusherMode(false);
                sleep(shootWait);
            }
            roboto.autoStopShoot();

            drive.followTrajectory(shoot2ToParkB);
            // roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
            sleep(1500);
        }
        else if (rings == 4) { // Zone C
            drive.followTrajectory(startToSwerve);
            drive.followTrajectory(swerveToZoneC);

            roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
            sleep(500);
            roboto.setWobbleGrabberMode(false);
            sleep(500);

            drive.followTrajectory(zoneCToShoot1);

            // shoot 3x + 3
            roboto.autoStartShoot(shooterAngle, vel);
            sleep(1100);
            roboto.setRingBlockerMode(false);
            for (int i = 0; i < shootCount + 3; i++) {
                roboto.setRingPusherMode(true);
                sleep(shootWait);
                roboto.setRingPusherMode(false);
                sleep(shootWait);
            }
            roboto.autoStopShoot();

            drive.followTrajectory(shoot1ToSuck1C);

            roboto.setIntakeMode(true);

            sleep(100);

            drive.followTrajectory(suck1ToSuck2C);

            sleep(100);

            roboto.setIntakeMode(false);

            drive.followTrajectory(suck2ToShoot2C);

            // shoot 3x
            roboto.autoStartShoot(shooterAngle, vel);
            sleep(1100);
            roboto.setRingBlockerMode(false);
            for (int i = 0; i < shootCount; i++) {
                roboto.setRingPusherMode(true);
                sleep(shootWait);
                roboto.setRingPusherMode(false);
                sleep(shootWait);
            }
            roboto.autoStopShoot();

            drive.followTrajectory(shoot2ToParkC);
            // roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
            sleep(1500);
        }
    }
}