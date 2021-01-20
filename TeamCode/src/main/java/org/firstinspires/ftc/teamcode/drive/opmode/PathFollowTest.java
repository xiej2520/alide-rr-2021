package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class PathFollowTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                /*
                .splineTo(new Vector2d(15, 10), Math.toRadians(90))
                .splineTo(new Vector2d(0, 20), Math.toRadians(180))
                .splineTo(new Vector2d(-15, 10), Math.toRadians(270))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .splineTo(new Vector2d(15, -10), Math.toRadians(270))
                .splineTo(new Vector2d(0, -20), Math.toRadians(180))
                .splineTo(new Vector2d(-15, -10), Math.toRadians(90))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                */
                .splineTo(new Vector2d(20, 0), Math.toRadians(0))
                .splineTo(new Vector2d(30, 20), Math.toRadians(90))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(30, -20))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(20, 0), Math.toRadians(180))
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
}