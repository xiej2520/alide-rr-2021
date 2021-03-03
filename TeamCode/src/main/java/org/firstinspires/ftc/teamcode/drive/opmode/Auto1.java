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

    public static Vector2d zoneA = new Vector2d(12.0, 60.0);
    public static Vector2d launchPos = new Vector2d(0.0, 36.0);
    public static Vector2d rings = new Vector2d(-36, 36);
    public static Vector2d launchLine = new Vector2d(12, 36);

    public static double shooterAngle = .3;
    public static double vel = -1200;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot roboto = new Robot(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 48, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        dashboard = roboto.drive.dashboard;

        Trajectory startToZoneA = drive.trajectoryBuilder(new Pose2d(), true) // moves around rings and then positions on zone A so the wobble goal can be delivered
                .splineTo(zoneA, Math.toRadians(180))
                .build();

        Trajectory launchToRing = drive.trajectoryBuilder(new Pose2d()) // move to rings to pick up
                .lineTo(rings)
                .build();

        Trajectory toLaunchPos = drive.trajectoryBuilder(new Pose2d()) // move to launch position
                .lineTo(launchPos)
                .build();

        Trajectory toLaunchLine = drive.trajectoryBuilder(new Pose2d()) // move to launch line
                .lineTo(launchLine)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(startToZoneA);

            // open wobble goal method

            drive.followTrajectory(toLaunchPos);

            drive.turn(Math.toRadians(180));

            // shoot 5x
            roboto.autoStartShoot(shooterAngle, vel);
            for (int i = 0; i < 5; i++) {
                roboto.setRingPusherMode(true);
                sleep(100);
                roboto.setRingPusherMode(false);
                sleep(100);
            }
            roboto.autoStopShoot();

            drive.turn(Math.toRadians(180));

            roboto.setIntakeMode(true);

            drive.followTrajectory(launchToRing);

            roboto.setIntakeMode(false);

            drive.turn(Math.toRadians(180));

            drive.followTrajectory(toLaunchPos);

            // shoot 5x
            roboto.autoStartShoot(shooterAngle, vel);
            for (int i = 0; i < 5; i++) {
                roboto.setRingPusherMode(true);
                sleep(100);
                roboto.setRingPusherMode(false);
                sleep(100);
            }
            roboto.autoStopShoot();

            drive.followTrajectory(toLaunchLine);
        }
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