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

@Autonomous(name="Blue Loading (Bad Al.)", group="Linear Opmode") // Assuming our teammate will do nothing
public class SafeLoadingZoneAuto5361 extends LinearOpMode {
    // Declare OpMode members.
    // public boolean isBlueAlliance = true; //Set to false if red alliance
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor, rightMotor;
    private Servo servoFR, servoFL, servoBR, servoBL, clawUpDown;

    public boolean getIsBlueAlliance() {return true;}

    @Override
    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        telemetry.addData("Status","Toward stone");
        telemetry.update();
        clawUpDown.setPosition(.1);
        servoBL.setPosition(.13);
        servoBR.setPosition(.1);
        servoFL.setPosition(.16); //may need adjusting
        servoFR.setPosition(.25);
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(1000);

        telemetry.addData("Status", "Grabbing stone");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(800);
        servoFL.setPosition(.34);
        servoFR.setPosition(.47);
        sleep(500);
        clawUpDown.setPosition(.7);
        sleep(800);

        telemetry.addData("Status","Turning back");
        telemetry.update();
        leftMotor.setPower(-1);
        rightMotor.setPower(.1);
        sleep(1050);

        telemetry.addData("Status", "Crossing bridge");
        telemetry.update();
        leftMotor.setPower(0.7);
        rightMotor.setPower(0.7);
        sleep(2700);

        telemetry.addData("Status", "Turning");
        telemetry.update();
        leftMotor.setPower(0.7);
        rightMotor.setPower(-0.7);
        sleep(600);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(300);

        telemetry.addData("Status", "Release");
        telemetry.update();
        clawUpDown.setPosition(.1);
        sleep(300);
        servoFL.setPosition(.22);
        servoFR.setPosition(.30);
        sleep(500);

        //the following until end is taken from building zone

        telemetry.addData("Status", "Grabbing foundation");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(300);
        servoBL.setPosition(.78);
        servoBR.setPosition(.75);
        sleep(500);

        telemetry.addData("Status", "Pull F");
        leftMotor.setPower(.6);
        rightMotor.setPower(.8);
        sleep(1000);

        telemetry.addData("Status", "Turning F");
        telemetry.update();
        leftMotor.setPower(-1);
        rightMotor.setPower(1);
        sleep(7000);

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
