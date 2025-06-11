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

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
            if (gamepad2.right_bumper) {
                robot.lifts.Bextendo_out();
            }
            else if (gamepad2.left_bumper) {
                robot.lifts.Bextendo_in();
            }

        }
    }
}