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

@Autonomous(name="Building Zone Autonomous", group="Linear Opmode")
public class BuildingZoneAuto5361 extends LinearOpMode {
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

        //Goal: Move the foundation (probably go around and push it) to the building site (10 points).
        //At the end, try to park over midfield tape (5 points).

        //Robot starts facing backward with the middle of the robot aligned with the middle of the foundation.
        telemetry.addData("Status","Toward foundation");
        telemetry.update();
        servoFL.setPosition(.8);
        servoFR.setPosition(.9);
        leftMotor.setPower(-.7);
        rightMotor.setPower(-.7);
        sleep(1000);

        telemetry.addData("Status", "Grabbing foundation");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(300);
        servoBL.setPosition(.78);
        servoBR.setPosition(.75);
        sleep(643); //(not Siri recommended)

        telemetry.addData("Status", "Pulling foundation");
        telemetry.update();
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        sleep(1000); //tentative
        leftMotor.setPower(0);
        sleep(1000);

        telemetry.addData("Status", "Pushing foundation");
        telemetry.update();
        rightMotor.setPower(0);
        sleep(300);
        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
        sleep(1500);

        telemetry.addData("Status", "Releasing foundation");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        servoBL.setPosition(.13);
        servoBR.setPosition(.1);
        sleep(643);
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(1500);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        /* Old autonomous (go around and push)
        telemetry.addData("Status", "Turning");
        telemetry.update();
        sleep(150);
        leftMotor.setPower(.5);
        rightMotor.setPower(-.5);
        sleep(700);

        telemetry.addData("Status", "Pulling out");
        telemetry.update();
        leftMotor.setPower(.5); */
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
        clawUpDown = hardwareMap.servo.get("clawUpDown");

        //switch these if the robot is going backward
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        servoFL.setDirection(Servo.Direction.FORWARD);
        servoFR.setDirection(Servo.Direction.REVERSE);
        servoBL.setDirection(Servo.Direction.REVERSE);
        servoBR.setDirection(Servo.Direction.FORWARD);
        clawUpDown.setDirection(Servo.Direction.REVERSE);
    }
}
