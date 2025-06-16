package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.Components.Claw;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;


@TeleOp(name="The Only TeleOP", group="Linear OpMode")
public class TeleOP extends LinearOpMode {
    //Button states
    private boolean b1state = false;
    private boolean b2state = false;
    private boolean y1state = false;
    private boolean x2state = false;
    private boolean a1state = false;
    private boolean a3state = false;
    private boolean a4state = false;
    private boolean xstate = false;
    private boolean dpadLeftState = false;
    private boolean bdpadUpState = false;
    private ElapsedTime mStateTime;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int k_state = 0;

        waitForStart();
        robot.init();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //driver hub output
            telemetry.addData("Lift position", robot.lifts.getLiftPosition());
            telemetry.addData("Extendo position", robot.lifts.getExtendoPosition());
            telemetry.update();

            //drivetrain control
            robot.mecnum.driveRobot(gamepad1);

            // vertical lift control
            if (gamepad2.right_bumper) {
                robot.lifts.raiseLift();
            }
            else if (gamepad2.left_bumper) {
                robot.lifts.lowerLift();
            }
            else {
                robot.lifts.stopLiftVertical();
            }

            //horizontal extendo control
            if (gamepad1.right_bumper) {
                robot.lifts.Bextendo_out();
            }
            else if (gamepad1.left_bumper) {
                robot.lifts.Bextendo_in();
            }

            if (gamepad1.a && !a1state) {
                a1state = true;
                k_state = 0;
                mStateTime.reset();
            }

            //grab or intake
            if (a1state) {
                switch (k_state) {
                    case 0:
                        robot.intake.fourBarIntaking();
                        robot.intake.clawIntakeOpen();
                        robot.intake.pitchIntaking();
                        if (mStateTime.seconds() >= 0.4) {
                            mStateTime.reset();
                            k_state++;
                        }
                        break;

                    case 1:
                        robot.intake.clawIntakeClose();
                        if (mStateTime.seconds() >= 0.25) {
                            mStateTime.reset();
                            k_state++;
                        }
                        break;

                    case 2:
                        robot.intake.pitchIntakeReady();
                        a1state = false;
                        break;
                }
            }

        }
    }
}