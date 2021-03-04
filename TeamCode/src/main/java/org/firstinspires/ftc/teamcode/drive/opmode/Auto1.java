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

    public static Pose2d start = new Pose2d(-60, 48, Math.toRadians(180));
    public static Pose2d zoneA = new Pose2d(12.0, 60.0, Math.toRadians(180));
    public static Pose2d launchPos = new Pose2d(0.0, 36.0, Math.toRadians(180));
    public static Pose2d launchPos2 = new Pose2d(0.0, 36.0, Math.toRadians(0));
    public static Pose2d rings = new Pose2d(-36, 36, Math.toRadians(0));
    public static Pose2d launchLine = new Pose2d(12, 36, Math.toRadians(0));

    public static Vector2d startV = new Vector2d(-60, 48);
    public static Vector2d zoneAV = new Vector2d(12.0, 60.0);
    public static Vector2d launchPosV = new Vector2d(0.0, 36.0);
    public static Vector2d ringsV = new Vector2d(-36, 36);
    public static Vector2d launchLineV = new Vector2d(12, 36);

    public static double shooterAngle = .3;
    public static double vel = -1200;

    public static int shootCount = 5;
    public static int shootWait = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot roboto = new Robot(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 48, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        dashboard = roboto.drive.dashboard;

        Trajectory startToZoneA = drive.trajectoryBuilder(start) // moves around rings and then positions on zone A so the wobble goal can be delivered
                .splineToConstantHeading(zoneAV, Math.toRadians(180))
                .build();

        Trajectory zoneAToLaunchPos = drive.trajectoryBuilder(zoneA) // moves around rings and then positions on zone A so the wobble goal can be delivered
                .splineToLinearHeading(launchPos, Math.toRadians(0))
                .build();

        Trajectory launchToRing = drive.trajectoryBuilder(launchPos2) // move to rings to pick up
                .splineToConstantHeading(ringsV, Math.toRadians(180))
                .build();

        Trajectory ringsToLaunchPos = drive.trajectoryBuilder(rings) // move to launch position
                .splineToConstantHeading(launchPosV, Math.toRadians(0))
                .build();

        Trajectory launchPosToLaunchLine = drive.trajectoryBuilder(launchPos2) // move to launch line
                .splineToConstantHeading(launchLineV, Math.toRadians(0))
                .build();

        waitForStart();

        drive.followTrajectory(startToZoneA);

        // open wobble goal method

        drive.followTrajectory(zoneAToLaunchPos);

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

/* ADD TO ROBOT.JAVA

public void autoStartShoot(double pos, double v) {
    setShooterAngle(pos);
    setShooterVelocity(v);
    setRingBlockerMode(false);
}

public void autoStopShoot() {
    setShooterAngle(0);
    setShooterVelocity(0);
    setRingBlockerMode(true);
}


 */