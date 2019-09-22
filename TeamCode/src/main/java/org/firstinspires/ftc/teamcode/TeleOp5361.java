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
    private Servo markerDrop;
    private String driveMode = "Tank Control"; //Values are "Tank Control" and "Joystick Control"

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
            landerRiser.setPower(upPower + downPower); //To stop, let go of both. If that doesn't work, hold both all the way down, but this is not preferable.

        }
    }

    private void setUp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        landerRiser = hardwareMap.get(DcMotor.class, "lander riser");
        markerDrop = hardwareMap.servo.get("marker");
        //switch these if the robot is going backward
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void omniCalc() //turns the gamepad controls into omniwheel commands
    {
        double leftPower;
        double rightPower;

        //Assign values to leftPower and rightPower
        telemetry.addData("Drive Mode", driveMode);
        telemetry.update();
        if (driveMode == "Tank Control") {
            leftPower = -gamepad1.left_stick.y;
            rightPower = -gamepad1.right_stick.y;
        }
        else if (driveMode == "Joystick Control") {
            leftPower = Range.clip(-gamepad1.right_stick.y + gamepad1.right_stick.x, -1, 1);
            rightPower = Range.clip(-gamepad1.right_stick.y - gamepad2.right_stick.x, -1, 1);
        }
        else {telemetry.addData("Error", "unacceptable driveMode"); telemetry.update(); leftPower = 0; rightPower = 0;}

        // write the values to the motors
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        //What does this do?
        String teleFormat = "leftPower (%.2f), rightPower (%.2f)";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", teleFormat, leftPower, rightPower);
        telemetry.update();
    }
}