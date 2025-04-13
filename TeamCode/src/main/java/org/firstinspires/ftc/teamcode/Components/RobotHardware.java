package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class RobotHardware {
    public LinearOpMode myOpMode = null;
    public RobotHardware(LinearOpMode opMode)
    {
        myOpMode = opMode;
    }
public Servo clawServo;
        public Servo wristServo;

        public DcMotorEx wristMotor;
    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;
    public DcMotorEx lLift;
    public DcMotorEx rLift;

    //Subsystems
    public Mecnum mecnum = new Mecnum();
    public Claw claw = new Claw();
    public Lifts lifts = new Lifts();

    Component[] components = {mecnum, claw, lifts};

    public void init()
    {
        //init motors
        fLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = myOpMode.hardwareMap.get(DcMotorEx.class, "bRight");
        wristMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "wristMotor");
        lLift = myOpMode.hardwareMap.get(DcMotorEx.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotorEx.class, "rLift");

//init servos
        clawServo = myOpMode.hardwareMap.get(Servo.class, "clawServo");




        //initialize subsystems
        for (int i = 0; i < components.length; i++)
        {
            components[i].init(this);
        }
        myOpMode.telemetry.addData("status", "initialized");
        myOpMode.telemetry.update();
        myOpMode.waitForStart();
    }

}
