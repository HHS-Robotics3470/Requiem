package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.Components.Claw;

@TeleOp(name="Teleop", group="Linear OpMode")
public class TeleOP extends LinearOpMode {

    private boolean astate = false;
    private boolean isUp = false;
    private boolean wasYPressed = false;
    private boolean isMoving = false;
    private ElapsedTime moveTimer = new ElapsedTime();
    private final double MOVE_TIME = 0.25 ; // ~60ms for ~30 degrees with 117 RPM
    private ElapsedTime wristTimer = new ElapsedTime();
    private boolean wristMoving = false;
    private boolean isWristUp = false;
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        telemetry.addData("Status", "Initialized");        telemetry.update();

        // Wait for the game to start (driver presses START)



        waitForStart();
        robot.init();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Claw", robot.claw.getClawPos());
            telemetry.addData("Left Lift", robot.lifts.getlLifts());
            telemetry.addData("Right Lift", robot.lifts.getrLifts());
            robot.mecnum.driveRobot(gamepad1);




            if (gamepad1.a && !astate)
            {
                robot.claw.toggleClaw();
                astate = true;
            }
            else if (!gamepad1.a && astate) {
                astate = false;
            }



            //old toggle code
           // if (gamepad1.a){
             //   robot.claw.toggleClaw();
            //}
         //   if (gamepad1.a) {
           //     robot.claw.clawOpen();
            //}
           // if(gamepad1.b){
             //   robot.claw.clawClose();
            //}



            if (gamepad1.y && !wasYPressed && !isMoving) {
                isUp = !isUp;
                isMoving = true;
                moveTimer.reset();
                robot.wristMotor.setPower(isUp ? 1.0 : -1.0); // Go up or down
            }

            // Stop motor after MOVE_TIME has passed
            if (isMoving && moveTimer.seconds() >= MOVE_TIME) {
                robot.wristMotor.setPower(0);
                isMoving = false;
            }

            wasYPressed = gamepad1.y;


            if (gamepad1.dpad_up) {
                robot.lifts.raiseLift();
            } else if (gamepad1.dpad_down) {
               robot.lifts.lowerLift();
            }else
                robot.lifts.sLifts();
            telemetry.update();
        }
    }
}
