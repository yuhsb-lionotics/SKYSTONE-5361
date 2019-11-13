package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Loading Zone Autonomous", group="Linear Opmode")
public class LoadingZoneAuto5361 extends LinearOpMode {
    // Declare OpMode members.
    public final boolean isBlueAlliance = true; //Set to false if red alliance
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor, rightMotor;
    private Servo servoFR, servoFL, servoBR, servoBL, clawUpDown;

    @Override
    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        clawUpDown.setPosition(.035);
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(800);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        servoFR.setPosition(.42); //Maybe .1
        servoFL.setPosition(.3);
        sleep(400);
        leftMotor.setPower(-1);
        rightMotor.setPower(-.1);
        sleep(1000);
        leftMotor.setPower(0.7);
        rightMotor.setPower(0.7);
        sleep(800);

    }

    private void setUp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (isBlueAlliance) {
            leftMotor = hardwareMap.dcMotor.get("leftMotor");
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
        } else {
            rightMotor = hardwareMap.dcMotor.get("leftMotor");
            leftMotor = hardwareMap.dcMotor.get("rightMotor");
        }
        servoFL = hardwareMap.servo.get("servoFL");
        servoFR = hardwareMap.servo.get("servoFR");
        servoBL = hardwareMap.servo.get("servoBL");
        servoBR = hardwareMap.servo.get("servoBR");
        //switch these if the robot is going backward
        clawUpDown = hardwareMap.servo.get("clawUpDown");

        //switch these if the robot is going backward
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        servoFL.setDirection(Servo.Direction.FORWARD);
        servoFR.setDirection(Servo.Direction.REVERSE);
        servoBL.setDirection(Servo.Direction.FORWARD);
        servoBR.setDirection(Servo.Direction.REVERSE);
        clawUpDown.setDirection(Servo.Direction.REVERSE);
    }
}
