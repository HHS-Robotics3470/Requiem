package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;

@Autonomous(name = "AttemptNumero1", group = "Autonomous")  //Name Confusion NEEDS FIXES
public class AutoAttemptUno extends LinearOpMode {

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.init();

        if (opModeIsActive()) {

        }
    }
}
