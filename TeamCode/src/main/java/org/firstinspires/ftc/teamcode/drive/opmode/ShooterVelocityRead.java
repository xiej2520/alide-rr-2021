package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;


@Config
@Autonomous(group = "drive")
public class ShooterVelocityRead extends LinearOpMode {

    private FtcDashboard dashboard;
    public static double targetVelocity = 2000;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);


        Robot roboto = new Robot(hardwareMap);

        int shooterFrontPrev = roboto.shooterFront.getCurrentPosition();
        int shooterBackPrev = roboto.shooterBack.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        waitForStart();

        timer.reset();
        while(opModeIsActive()) {
            roboto.setShooterVelocity(targetVelocity);
            if (timer.time() > 25) {
                TelemetryPacket packet = new TelemetryPacket();
                double shooterFrontVel = (double) (roboto.shooterFront.getCurrentPosition() - shooterFrontPrev) / timer.time();
                double shooterBackVel = (double) (roboto.shooterBack.getCurrentPosition() - shooterBackPrev) / timer.time();

                packet.put("shooterFrontVel", shooterFrontVel);
                packet.put("shooterBackVel", shooterBackVel);
                dashboard.sendTelemetryPacket(packet);

                shooterFrontPrev = roboto.shooterFront.getCurrentPosition();
                shooterBackPrev = roboto.shooterBack.getCurrentPosition();
                timer.reset();
            }


        }




    }
}