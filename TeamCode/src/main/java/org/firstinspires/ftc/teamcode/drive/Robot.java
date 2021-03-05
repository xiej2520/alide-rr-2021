package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private Servo wobbleGrabber;

    /* Robot state variables */
    private boolean intakeMode;
    private boolean intakeDirection; // true:in, false:out
    private boolean ringBlockerMode;
    private boolean ringPusherMode;
    private double shooterAnglePos;
    private double shooterAngleDeg;
    private Pose2d vel;
    private double shooterVelocity;
    private boolean wobbleGrabberMode;

    /* Config variables */
    public static double ringBlockerOffPos = 0.23;
    public static double ringBlockerOnPos = 0.35;
    public static double ringPusherOnPos = 0.12;
    public static double ringPusherOffPos = 0.31;

    public static double shooterAngleMaxPos = 0.7; // ~0 degrees, almost hits plastic
    public static double shooterAngleMinPos = 0.07; // 90 degrees up
    public static double shooterAngleMinDeg = 0;
    public static double shooterAngleMaxDeg = 90;

    public static double wobbleGrabberPosMax = 0.85;
    public static double wobbleGrabberPosMin = 0.3;


    public Robot(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");
        ringBlocker = hardwareMap.get(Servo.class, "ringBlocker");
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");

        intake1.setDirection(DcMotor.Direction.REVERSE); // intake motors need to be opposite directions
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMode = false;
        intakeDirection = true;
        ringBlockerMode = true;
        ringPusherMode = false;
        vel = new Pose2d(0, 0, 0);

        setRingBlockerMode(true);
        setRingPusherMode(false);

        shooterAnglePos = shooterAngleMaxPos;
        shooterAngleDeg = 0;
        setShooterAngle(shooterAngleMaxPos);
        shooterVelocity = 0;

    }

    public boolean getIntakeMode() {
        return intakeMode;
    }
    public void setIntakeMode(boolean mode) {
        if (mode == true) {
            intakeMode = true;
            if (intakeDirection == true) {
                intake1.setPower(1);
                intake2.setPower(1);
            }
            else {
                intake1.setPower(-1);
                intake2.setPower(-1);
            }
        }
        else {
            intakeMode = false;
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
    public boolean getIntakeDirection() { return intakeDirection; }
    public void setIntakeDirection(boolean direction) {
        intakeDirection = direction;
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

    public double getShooterVelocity() {
        return shooterVelocity;
    }
    public void setShooterVelocity(double v) {
        shooter1.setVelocity(v);
        shooter2.setVelocity(v);
        this.shooterVelocity = v;
    }

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

    public boolean getWobbleGrabberMode() {
        return this.wobbleGrabberMode;
    }
    public void setWobbleGrabberMode(boolean mode) {
        if (mode == true) {
            wobbleGrabber.setPosition(wobbleGrabberPosMax);
            wobbleGrabberMode = true;
        }
        else {
            wobbleGrabber.setPosition(wobbleGrabberPosMin);
            wobbleGrabberMode = false;
        }
    }

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

}