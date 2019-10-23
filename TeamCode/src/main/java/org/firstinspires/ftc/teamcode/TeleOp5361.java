package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver controlled", group="Linear Opmode")
@Disabled
public class TeleOp5361 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor, rightMotor, landerRiser;
    private Servo servoFR, servoFL, servoBR, servoBL, clawUpDown;
    private String driveMode = "Tank Control"; //Values are "Tank Control" and "Joystick Control".
                                               //Press Y on the controller to change the mode.
    private boolean yWasPressed = false;

    @Override
    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            omniCalc();
            //Lander Riser:
            double upPower = gamepad1.right_trigger;
            double downPower = -gamepad1.left_trigger;
            //landerRiser.setPower(upPower + downPower); //To stop, let go of both. If that doesn't work, hold both all the way down, but this is not preferable.
            //idle();
        }
    }

    private void setUp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        //landerRiser = hardwareMap.get(DcMotor.class, "lander riser");
        servoFL = hardwareMap.servo.get("servoFL");
        servoFR = hardwareMap.servo.get("servoFR");
        servoBL = hardwareMap.servo.get("servoBL");
        servoBR = hardwareMap.servo.get("servoBR");
        clawUpDown = hardwareMap.servo.get("clawUpDown");

        //switch these if the robot is going backward
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        servoFL.setDirection(Servo.Direction.FORWARD);
        servoFR.setDirection(Servo.Direction.REVERSE);
        servoBL.setDirection(Servo.Direction.FORWARD);
        servoBR.setDirection(Servo.Direction.REVERSE);
        clawUpDown.setDirection(Servo.Direction.FORWARD);
    }

    private void omniCalc() //turns the gamepad controls into omniwheel commands
    {
        double leftPower;
        double rightPower;

     /* if (!yWasPressed & gamepad1.y) {
            if (driveMode == "Tank Control") {driveMode = "Joystick Control";}
            else if (driveMode == "Joystick Control") {driveMode = "Tank Control";}
            else {telemetry.addData("Error", "Unacceptable driveMode");}
        }
        yWasPressed = gamepad1.y; */
        telemetry.addData("Drive Mode", driveMode);
        telemetry.update();
        //Assign values to leftPower and rightPower
        if (driveMode == "Tank Control") {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        }/* else if (driveMode == "Joystick Control") {
            leftPower = Range.clip(-gamepad1.right_stick_y + gamepad1.right_stick_x, -1, 1);
            rightPower = Range.clip(-gamepad1.right_stick_y - gamepad2.right_stick_x, -1, 1);
        } */ else {telemetry.addData("Error", "Unacceptable driveMode"); telemetry.update(); leftPower = 0; rightPower = 0;}

        // write the values to the motors
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (gamepad1.right_bumper) {servoBL.setPosition(.1); servoBR.setPosition(.1);}
        if (gamepad1.left_bumper) {servoBL.setPosition(.7); servoBR.setPosition(.7);}
        if (gamepad1.b) {servoFL.setPosition(0); servoFR.setPosition(0);}
        if (gamepad1.x) {servoFL.setPosition(.2); servoFR.setPosition(.2);}
        if (gamepad1.y) {clawUpDown.setPosition(0.5);}
        if (gamepad1.a) {clawUpDown.setPosition(0);}

        //What does this do?
        String teleFormat = "leftPower (%.2f), rightPower (%.2f)";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", teleFormat, leftPower, rightPower);
        telemetry.update();
    }
}