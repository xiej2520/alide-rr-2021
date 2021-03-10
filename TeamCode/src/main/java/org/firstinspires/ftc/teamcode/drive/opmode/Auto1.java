package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class Auto1 extends LinearOpMode {

    private FtcDashboard dashboard;

    public static Pose2d startP = new Pose2d(-60, -48, Math.toRadians(180));
    public static Pose2d launchPosP = new Pose2d(12, -48, Math.toRadians(180));

    public static Vector2d zoneAV = new Vector2d(6, -60);
    public static Vector2d middleStepV = new Vector2d(-24, -60);
    public static Vector2d launchPosV = new Vector2d(12, -48);
    // public static Vector2d ringsV = new Vector2d(-36, -48); // ????????????
    public static Vector2d launchLineV = new Vector2d(48, -36);

    public static double shooterAngle = 21;
    public static double vel = 1700;
    public static double turnBeforeShoot = 180;
    public static double turnBeforeRings = 135;
    public static double turnBeforeShoot2 = 180;
    public static double extraAdjustment = 0;

    public static int shootCount = 3;
    public static int shootWait = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot roboto = new Robot(hardwareMap);

        Pose2d startPose = new Pose2d(-60, -48, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        dashboard = roboto.drive.dashboard;

        Trajectory startToZoneA1 = drive.trajectoryBuilder(startP)
                .lineToConstantHeading(middleStepV)
                .build();

        Trajectory startToZoneA2 = drive.trajectoryBuilder(startToZoneA1.end())
                .lineToConstantHeading(zoneAV)
                .build();

        Trajectory zoneAToLaunchPos = drive.trajectoryBuilder(startToZoneA2.end())
                .splineToLinearHeading(launchPosP, Math.toRadians(0))
                .build();

        Trajectory launchToRing = drive.trajectoryBuilder(zoneAToLaunchPos.end())
                .forward(36)
                .build();

        Trajectory ringsToLaunchPos = drive.trajectoryBuilder(launchToRing.end()
                .plus(new Pose2d(0, 0, Math.toRadians(turnBeforeShoot2))))
                .splineTo(launchPosV, Math.toRadians(0))
                .build();

        Trajectory launchPosToLaunchLine = drive.trajectoryBuilder(ringsToLaunchPos.end())
                .splineToConstantHeading(launchLineV, Math.toRadians(0))
                .build();

        waitForStart();

        drive.followTrajectory(startToZoneA1);
        drive.followTrajectory(startToZoneA2);

        roboto.setShooterAngleDeg(roboto.shooterAngleDegMax);
        sleep(500);
        roboto.setWobbleGrabberMode(false);
        sleep(500);

        drive.followTrajectory(zoneAToLaunchPos);

        drive.turn(Math.toRadians(turnBeforeShoot));

        // shoot 5x
        roboto.autoStartShoot(shooterAngle, vel);
        sleep(1250);
        roboto.setRingBlockerMode(false);
        for (int i = 0; i < shootCount; i++) {
            roboto.setRingPusherMode(true);
            sleep(shootWait);
            roboto.setRingPusherMode(false);
            sleep(shootWait);
        }
        roboto.autoStopShoot();

        drive.turn(Math.toRadians(turnBeforeRings));

        roboto.setIntakeMode(true);

        sleep(500);

        drive.followTrajectory(launchToRing);

        sleep(500);

        roboto.setIntakeMode(false);

        drive.turn(Math.toRadians(turnBeforeShoot2));

        drive.followTrajectory(ringsToLaunchPos);

        drive.turn(Math.toRadians(180 - turnBeforeRings - extraAdjustment));

        // shoot 5x
        roboto.autoStartShoot(shooterAngle, vel);
        sleep(1250);
        roboto.setRingBlockerMode(false);
        for (int i = 0; i < shootCount; i++) {
            roboto.setRingPusherMode(true);
            sleep(shootWait);
            roboto.setRingPusherMode(false);
            sleep(shootWait);
        }
        roboto.autoStopShoot();

        drive.followTrajectory(launchPosToLaunchLine);
    }
}