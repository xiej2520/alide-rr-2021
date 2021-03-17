package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;

import java.util.Objects;


@Config
@Autonomous(group = "drive")
public class ShooterVelocityRead extends LinearOpMode {

    private FtcDashboard dashboard;
    public static double targetVelocity = 5000;
    public static double RUNTIME = 4.0;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);


        Robot roboto = new Robot(hardwareMap);

        int shooterFrontPrev = roboto.shooterFront.getCurrentPosition();
        int shooterBackPrev = roboto.shooterBack.getCurrentPosition();

        double[] shooterFrontValues = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        double[] shooterBackValues = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int c = 0;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime runTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        waitForStart();
        timer.reset();
        runTimer.reset();
        while (!isStopRequested() && runTimer.seconds() < RUNTIME) {
            roboto.setShooterVelocity(targetVelocity);
            // setVelocity units is ticks/second
            // ticks/second reading / (ticks/revolution) = revolutions/second
            // shooter motor has 28 Encoder Countable Events Per Revolution (Output Shaft)
            if (timer.time() > 100) {
                TelemetryPacket packet = new TelemetryPacket();
                double shooterFrontVel = 1000 * (double) (roboto.shooterFront.getCurrentPosition() - shooterFrontPrev) / timer.time();
                double shooterBackVel =  1000 * (double) (roboto.shooterBack.getCurrentPosition() - shooterBackPrev) / timer.time();
                shooterFrontValues[c] = shooterFrontVel;
                shooterBackValues[c] = shooterBackVel;
                c = (c+1) % 10;

                shooterFrontPrev = roboto.shooterFront.getCurrentPosition();
                shooterBackPrev = roboto.shooterBack.getCurrentPosition();

                packet.put("shooterFrontVel (ticks/s)", shooterFrontVel);
                packet.put("shooterBackVel (ticks/s)", shooterBackVel);
                packet.put("shooterFrontVel (rev/s)", shooterFrontVel/28);
                packet.put("shooterBackVel (rev/s)", shooterBackVel/28);
                dashboard.sendTelemetryPacket(packet);

                timer.reset();
            }
        }
        roboto.setShooterVelocity(0);
        double shooterFrontAvg = 0;
        double shooterBackAvg = 0;
        for (int i=0; i<10; i++) {
            shooterFrontAvg += shooterFrontValues[i];
            shooterBackAvg += shooterBackValues[i];
        }
        shooterFrontAvg /= 10;
        shooterBackAvg /= 10;
        telemetry.addData("Shooter Front Max Velocity (ticks/s): ", shooterFrontAvg);
        telemetry.addData("Shooter Back Max Velocity (ticks/s): ", shooterBackAvg);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }
}