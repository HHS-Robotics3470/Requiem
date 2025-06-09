package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecnum implements Component{
    private RobotHardware robotHardware;

    public final double DRIVE_SPEED_MAX = 0.95;
    public final double DRIVE_SPEED_SLOW = 0.05;
    public double driveSpeedControl = DRIVE_SPEED_MAX;

    public double speedFLeft = 1.0;
    public double speedFRight = 1.0;
    public double speedBLeft = 1.0;
    public double speedBRight = 1.0;

    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;


    @Override
    public void init(HardwareMap hardwareMap) {
        fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        fLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fRight.setDirection(DcMotorEx.Direction.FORWARD);
        bLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bRight.setDirection(DcMotorEx.Direction.FORWARD);

        fLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        stopAllMotors();  // Ensure motors are off after initialization
    }

    public void stopAllMotors() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    public void setDrivePowerFLeft(double power) {
        fLeft.setPower(power * driveSpeedControl * speedFLeft);
    }

    public void setDrivePowerFRight(double power) {
        fRight.setPower(power * driveSpeedControl * speedFRight);
    }

    public void setDrivePowerBLeft(double power) {
        bLeft.setPower(power * driveSpeedControl * speedBLeft);
    }

    public void setDrivePowerBRight(double power) {
        bRight.setPower(power * driveSpeedControl * speedBRight);
    }

    public void setDrivePowerAuto(double v1, double v2, double v3, double v4) {
        setDrivePowerFLeft(v1);
        setDrivePowerFRight(v2);
        setDrivePowerBLeft(v3);
        setDrivePowerBRight(v4);
    }

    public void setDrivePower(double v1, double v2, double v3, double v4) {
        fLeft.setPower(v1 * driveSpeedControl);
        fRight.setPower(v2 * driveSpeedControl);
        bLeft.setPower(v3 * driveSpeedControl);
        bRight.setPower(v4 * driveSpeedControl);
    }

    public void powerFunction(double speedF, double speedR, double speedB, double speedFL) {
        this.speedFLeft = speedFL;
        this.speedFRight = speedF;
        this.speedBLeft = speedB;
        this.speedBRight = speedR;
    }

    public void driveRobot(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Apply deadzone for all joystick axes
        y = Math.abs(y) > 0.10 ? y : 0;  // Increased deadzone
        x = Math.abs(x) > 0.10 ? x : 0;
        rx = Math.abs(rx) > 0.10 ? rx : 0;

        // Check if joystick is at rest
        if (y == 0 && x == 0 && rx == 0) {
            stopAllMotors();  // Stop all motors if no input
        } else {
            double frLeft = y + x + rx;
            double frRight = y - x - rx;
            double baLeft = y - x + rx;
            double baRight = y + x - rx;

            setDrivePower(frLeft, frRight, baLeft, baRight);
        }
    }

    public void brake(double button) {
        // Optional brake logic you can implement later
    }

    private void moveForward(double power, int time) {
        setDrivePower(power, power, power, power);
        stopAllMotors();
    }

    public void strafe(double power, int time) {
        fLeft.setPower(power * driveSpeedControl * speedFLeft);
        fRight.setPower(-power * driveSpeedControl * speedFRight);
        bLeft.setPower(-power * driveSpeedControl * speedBLeft);
        bRight.setPower(power * driveSpeedControl * speedBRight);
        stopAllMotors();
    }

    public void rotate(double power, int time) {
        fLeft.setPower(power * driveSpeedControl * speedFLeft);
        fRight.setPower(-power * driveSpeedControl * speedFRight);
        bLeft.setPower(power * driveSpeedControl * speedBLeft);
        bRight.setPower(-power * driveSpeedControl * speedBRight);
        stopAllMotors();
    }
}

