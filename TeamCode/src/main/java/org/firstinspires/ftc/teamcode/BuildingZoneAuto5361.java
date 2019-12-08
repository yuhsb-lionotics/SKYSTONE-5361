package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Building Zone Auto", group="Linear Opmode")
public class BuildingZoneAuto5361 extends LinearOpMode {
    // Declare OpMode members.
    // public boolean isBlueAlliance = true; //Set to false if red alliance
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor, rightMotor;
    private Servo servoFR, servoFL, servoBR, servoBL, clawUpDown;

    public boolean getIsBlueAlliance() {return true;}


    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        //Goal: Move the foundation (probably go around and push it) to the building site (10 points).
        //At the end, try to park over midfield tape (5 points).

        //Robot starts facing backward with the left edge of the robot aligned with the left edge of the foundation (on the blue alliance).
        telemetry.addData("Status","Toward foundation");
        telemetry.update();
        servoFL.setPosition(.8);
        servoFR.setPosition(.9);
        leftMotor.setPower(-.7);
        rightMotor.setPower(-.7);
        sleep(1200);

        telemetry.addData("Status", "Grabbing foundation");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(300);
        servoBL.setPosition(.78);
        servoBR.setPosition(.75);
        sleep(500);

        telemetry.addData("Status", "Pull F");
        leftMotor.setPower(.7);
        rightMotor.setPower(.7);
        sleep(1000);

        telemetry.addData("Status", "Turning F");
        telemetry.update();
        leftMotor.setPower(-1);
        rightMotor.setPower(1);
        sleep(6000);

        telemetry.addData("Status", "Unlatch");
        servoBL.setPosition(.13);
        servoBR.setPosition(.10);
        sleep(300);


        telemetry.addData("Status", "Pushing foundation");
        telemetry.update();
        leftMotor.setPower(-.8);
        rightMotor.setPower(-.8);
        sleep(2000);

        telemetry.addData("Status", "Robot park under bridge");
        telemetry.update();
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(1000);

        telemetry.addData("Dinner", "Served <0/");
        telemetry.update();
/*
        telemetry.addData("Status", "Repositioning");
        rightMotor.setPower(.5);
        sleep(400); //angle
        leftMotor.setPower(.5);
        sleep(600); //distance
        rightMotor.setPower(-.5);
        sleep(300); //twisting
        leftMotor.setPower(-.5);
        sleep(450); //returning to foundation*/
/*
        telemetry.addData("Status", "Releasing foundation");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(643);
        //May have to turn a little so the robot doesn't crash
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(3000);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

 */
    }

    private void setUp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (getIsBlueAlliance()) {
            leftMotor = hardwareMap.dcMotor.get("leftMotor");
            rightMotor = hardwareMap.dcMotor.get("rightMotor");
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            rightMotor = hardwareMap.dcMotor.get("leftMotor");
            leftMotor = hardwareMap.dcMotor.get("rightMotor");
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        servoFL = hardwareMap.servo.get("servoFL");
        servoFR = hardwareMap.servo.get("servoFR");
        servoBL = hardwareMap.servo.get("servoBL");
        servoBR = hardwareMap.servo.get("servoBR");
        clawUpDown = hardwareMap.servo.get("clawUpDown");

        //switch these if the robot is going backward
        servoFL.setDirection(Servo.Direction.FORWARD);
        servoFR.setDirection(Servo.Direction.REVERSE);
        servoBL.setDirection(Servo.Direction.REVERSE);
        servoBR.setDirection(Servo.Direction.FORWARD);
        clawUpDown.setDirection(Servo.Direction.REVERSE);
    }
}
