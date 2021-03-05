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
    public static Pose2d launchPosP = new Pose2d(0.0, -36.0, Math.toRadians(180));

    public static Vector2d zoneAV = new Vector2d(12.0, -60.0);
    public static Vector2d middleStepV = new Vector2d(-24, -60.0);
    public static Vector2d launchPosV = new Vector2d(0.0, -36.0);
    public static Vector2d ringsV = new Vector2d(-36, -36);
    public static Vector2d launchLineV = new Vector2d(12, -36);

    public static double shooterAngle = 35;
    public static double vel = -2000;

    public static int shootCount = 3;
    public static int shootWait = 750;

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

        Trajectory launchToRing = drive.trajectoryBuilder(zoneAToLaunchPos.end()
                .plus(new Pose2d(0, 0, Math.toRadians(180)))) // robot turns before this traj
                .splineToConstantHeading(ringsV, Math.toRadians(180))
                .build();

        Trajectory ringsToLaunchPos = drive.trajectoryBuilder(launchToRing.end())
                .splineToConstantHeading(launchPosV, Math.toRadians(0))
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

        sleep(1000);
        drive.turn(Math.toRadians(180));

        // shoot 5x
        roboto.autoStartShoot(shooterAngle, vel);
        sleep(2000);
        for (int i = 0; i < shootCount; i++) {
            roboto.setRingPusherMode(true);
            sleep(shootWait);
            roboto.setRingPusherMode(false);
            sleep(shootWait);
        }
        roboto.autoStopShoot();

        drive.turn(Math.toRadians(180));

        roboto.setIntakeMode(true);

        sleep(5000); // stopping so I can debug

        drive.followTrajectory(launchToRing);

        roboto.setIntakeMode(false);

        drive.turn(Math.toRadians(180));

        drive.followTrajectory(ringsToLaunchPos);

        // shoot 5x
        roboto.autoStartShoot(shooterAngle, vel);
        sleep(2000);
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