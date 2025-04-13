package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw implements Component {

    private LinearOpMode myOpMode;
    private Servo clawServo;
    private DcMotorEx wristMotor;
    private ElapsedTime wristTimer = new ElapsedTime();
    private boolean wristMoving = false;
    private boolean isWristUp = false;

    private final double CLAW_OPEN_POSITION = 0.98;
    private final double CLAW_CLOSE_POSITION = 1.0;
    private final int WRIST_UP_POSITION = 0;
    private final int WRIST_DOWN_POSITION = 1;

    private boolean isClawOpen = false; // Track claw open/close state

    @Override
    public void init(RobotHardware robotHardware) {
        myOpMode = robotHardware.myOpMode;
        wristMotor = robotHardware.wristMotor;
        clawServo = robotHardware.clawServo;

        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Toggle claw open/close state


    public void toggleClaw() {
        isClawOpen = !isClawOpen;  // Toggle claw state

        if (isClawOpen) {
            clawServo.setPosition(CLAW_OPEN_POSITION);
        } else {
            clawServo.setPosition(CLAW_CLOSE_POSITION);
        }
    }
//open w a and close w b code
    public void clawOpen() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    public void clawClose() {
        clawServo.setPosition(CLAW_CLOSE_POSITION);
    }

    public double getClawPos() {
        return clawServo.getPosition();
    }


    // Toggle wrist position when 'Y' is pressed


}