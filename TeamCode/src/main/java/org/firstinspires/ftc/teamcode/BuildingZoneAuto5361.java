package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name="MOVE FORWARD LOL", group="Linear Opmode")
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
        sleep(1075);

        telemetry.addData("Dinner", "Served <0/");
        telemetry.update();
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
