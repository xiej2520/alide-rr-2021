package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Robot {
    public SampleMecanumDrive drive;


    /* Motors and Servos */
    private DcMotorEx intake1; // top roller (big wheels)
    private DcMotorEx intake2; // bottom roller (small wheels + ramp)
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private Servo shooterAngle;
    private Servo ringBlocker;
    private Servo ringPusher;

    /* Robot state variables */
    private boolean intakeMode;
    private boolean ringBlockerMode;
    private boolean ringPusherMode;
    private double shooterAnglePos;
    private double shooterAngleDeg;
    private Pose2d vel;

    /* Config variables */
    public static double ringBlockerOffPos = 0.23;
    public static double ringBlockerOnPos = 0.35;
    public static double ringPusherOnPos = 0.12;
    public static double ringPusherOffPos = 0.31;

    public static double shooterAngleMaxPos = 0.7; // ~0 degrees, almost hits plastic
    public static double shooterAngleMinPos = 0.07; // 90 degrees up
    public static double shooterAngleMinDeg = 0;
    public static double shooterAngleMaxDeg = 90;


    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");
        ringBlocker = hardwareMap.get(Servo.class, "ringBlocker");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");

        intake1.setDirection(DcMotor.Direction.REVERSE); // intake motors need to be opposite directions


        intakeMode = false;
        ringBlockerMode = true;
        ringPusherMode = false;
        vel = new Pose2d(0, 0, 0);

        setRingBlockerMode(true);
        setRingPusherMode(false);

        shooterAnglePos = shooterAngleMaxPos;
        shooterAngleDeg = 0;
        setShooterAngle(shooterAngleMaxPos);

    }

    public boolean getIntakeMode() {
        return intakeMode;
    }
    public void setIntakeMode(boolean mode) {
        if (mode == true) {
            intakeMode = true;
            intake1.setPower(1);
            intake2.setPower(1);
        }
        else {
            intakeMode = false;
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }

    public boolean getRingBlockerMode() {
        return ringBlockerMode;
    }
    public void setRingBlockerMode(boolean mode) {
        if (mode == true) {
            ringBlockerMode = true;
            ringBlocker.setPosition(ringBlockerOnPos);
        }
        else {
            ringBlockerMode = false;
            ringBlocker.setPosition(ringBlockerOffPos);
        }
    }

    public boolean getRingPusherMode() {
        return ringPusherMode;
    }
    public void setRingPusherMode(boolean mode) {
        if (mode == true) {
            ringPusherMode = true;
            ringPusher.setPosition(ringPusherOnPos);
        }
        else {
            ringPusherMode = false;
            ringPusher.setPosition(ringPusherOffPos);
        }
    }
    public Pose2d getVel() { return vel; }
    public void setVel(Pose2d v) {
        vel = v;
    }

    public void setShooterVelocity(double v) {
        shooter1.setVelocity(v);
        shooter2.setVelocity(v);
    }

    /*
    public void changeShooterAngle(int delta) {
        if (shooterAngle.getTargetPosition() + delta < shooterAngleMaxPos) {
            shooterAngle.setTargetPosition(shooterAngleMaxPos);
        }
        else if (shooterAngle.getTargetPosition() + delta > 0) {
            shooterAngle.setTargetPosition(0);
        }
        else {
            shooterAngle.setTargetPosition(shooterAngle.getCurrentPosition() + delta);
        }

        if (shooterAngle.getCurrentPosition() < shooterAngle.getTargetPosition()) {
            shooterAngle.setPower(0.1);
        }
        else if (shooterAngle.getCurrentPosition() > shooterAngle.getTargetPosition()) {
            shooterAngle.setPower(-0.1);
        }
        else {
            shooterAngle.setPower(0);
        }

        shooterAnglePos = shooterAngle.getCurrentPosition();
    }
    */
    public void changeShooterAngle(double delta) {
        if (shooterAnglePos + delta > shooterAngleMaxPos) {
            setShooterAngle(shooterAngleMaxPos);
        }
        else if (shooterAnglePos + delta < shooterAngleMinPos) {
            setShooterAngle(shooterAngleMinPos);
        }
        else {
            setShooterAngle(shooterAnglePos + delta);
        }
    }
    public double getShooterAnglePos() {
        return shooterAnglePos;
    }
    public double getShooterAngleDeg() {
        return shooterAngleDeg;
    }
    public void setShooterAngle(double pos) {
        shooterAnglePos = pos;
        shooterAngleDeg = linearMap(pos, shooterAngleMaxPos, shooterAngleMinPos, shooterAngleMinDeg, shooterAngleMaxDeg);
        shooterAngle.setPosition(pos);
    }
    public void setShooterAngleDeg(double angle) {
        setShooterAngle(linearMap(angle, shooterAngleMinDeg, shooterAngleMaxDeg, shooterAngleMaxPos, shooterAngleMinPos));
    }

    public double linearMap(double val, double oldMin, double oldMax, double newMin, double newMax) {
        return newMin + (newMax-newMin) / (oldMax-oldMin) * (val - oldMin);
    }

}