package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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
public class TeleOp5361 extends LinearOpMode {

    // Declare OpMode members. //RECODING FOR 6 MOTORS, 4 SERVOS
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL, motorFR, motorBL, motorBR, strafeMotor, clawTower, dude111;
    private Servo sClawR, sClawL, fGripR, fGripL, compressor; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left
    private ColorSensor leftColor, rightColor;
    private String driveMode = "Tank Control"; //Values are "Tank Control" and "Joystick Control".
                                               //Press Y on the controller to change the mode.
    //private boolean yWasPressed = false;
    //private String driverMode = "Power"; //shifts between "Power" and "Precision" motor controls //NO LONGER USING


    @Override
    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveCalc();
            roboCalc();
            telemetry.addData("Left Yellowness ratio",
                    (leftColor.red() + leftColor.green()) / (double) leftColor.blue());
            telemetry.addData("Right Yellowness ratio",
                    (rightColor.red() + rightColor.green()) / (double) rightColor.blue());
            if      (leftColor.argb() == 0) {telemetry.addData("Skystone", "Left");}
            else if (rightColor.argb() == 0) {telemetry.addData("Skystone", "Right");}
            else if (leftColor.red() + leftColor.green() < leftColor.blue() * 3.3) {
                telemetry.addData("Skystone", "Left"); }
            else if (rightColor.red() + rightColor.green() < rightColor.blue() * 3.3) {
                telemetry.addData("Skystone", "Right"); }
            else {telemetry.addData("Skystone", "Center");}
            telemetry.update();
            //idle();
        }
    }
    public static void openClaw(Servo leftServo, Servo rightServo) {
        leftServo.setPosition(.4);
        rightServo.setPosition(.5);
    }
    public static void grabStone(Servo leftServo, Servo rightServo) {
        leftServo.setPosition(.8);
        rightServo.setPosition(.9);
    }
    public static void closeClaw(Servo leftServo, Servo rightServo) { // all the way
        leftServo.setPosition(1);
        rightServo.setPosition(1);
    }
    public static void wideClaw(Servo leftServo, Servo rightServo){
        leftServo.setPosition(.15);
        rightServo.setPosition(.25);
    }

    private void setUp(){
        telemetry.addData("Status", "Initialized - Setting Up");
        telemetry.update();

        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        strafeMotor = hardwareMap.dcMotor.get("motorM");
        clawTower = hardwareMap.dcMotor.get("clawTower");
        dude111 = hardwareMap.dcMotor.get("tapeTongue");

        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");
        compressor = hardwareMap.servo.get("compressor");
        leftColor = hardwareMap.colorSensor.get("colorL");
        rightColor = hardwareMap.colorSensor.get("colorR");


        //switch these if they are going backward
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        strafeMotor.setDirection(DcMotor.Direction.FORWARD);
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        dude111.setDirection(DcMotor.Direction.FORWARD);

        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);
        compressor.setDirection(Servo.Direction.FORWARD);


        //resetting encoders & waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
/*
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    private void driveCalc() //turns the gamepad controls into omniwheel commands
    {
        double leftPower; double rightPower;

        /* if (!yWasPressed & gamepad1.y) {
            if (driveMode == "Tank Control") {driveMode = "Joystick Control";}
            else if (driveMode == "Joystick Control") {driveMode = "Tank Control";}
            else {telemetry.addData("Error", "Unacceptable driveMode");}
        }
        yWasPressed = gamepad1.y; */

        telemetry.addData("Drive Mode", driveMode);
        //Assign values to leftPower and rightPower
        if (driveMode.equals("Tank Control")) {
            leftPower       =   (-gamepad1.left_stick_y)*.7;
            rightPower      =   (-gamepad1.right_stick_y)*.7;
        }
     /*   else if (driveMode == "Joystick Control") {
            leftPower = Range.clip(-gamepad1.right_stick_y + gamepad1.right_stick_x, -1, 1);
            rightPower = Range.clip(-gamepad1.right_stick_y - gamepad2.right_stick_x, -1, 1);
        } */
        else {
            telemetry.addData("Error", "Unacceptable Drive Mode");
            leftPower = 0;
            rightPower = 0;
        }

        // write the values to the motors

        //Drive Controls [ALPHA]
        motorBL.setPower(leftPower); //joysticks power
        motorFL.setPower(leftPower);
        motorBR.setPower(rightPower);
        motorFR.setPower(rightPower);
        strafeMotor.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.7); //strafe

        String teleFormat = "leftPower (%.2f), rightPower (%.2f)";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", teleFormat, leftPower, rightPower);
    }

    private void roboCalc() {

        if      (gamepad1.x)    {TeleOp5361.grabStone(sClawL, sClawR);} //close
        else if (gamepad1.b)    {TeleOp5361.openClaw(sClawL, sClawR);} // open

        if      (gamepad2.dpad_up)   {fGripL.setPosition(.19); fGripR.setPosition(.19);} //up
        else if (gamepad2.dpad_down) {fGripL.setPosition(.78); fGripR.setPosition(.78);} //down

        /*if      (gamepad2.y)    {clawTower.setPower(1); } //tower up
        else if (gamepad2.a)    {clawTower.setPower(-1); } //tower down
        else if (!gamepad2.y||!gamepad2.a) {clawTower.setPower(0);} //tower idle*/
        double clawString = -gamepad2.right_stick_y;
        clawTower.setPower(clawString);

        double scotchTape = -gamepad2.left_stick_y;
        dude111.setPower(scotchTape);

        /*
        //testing values for the servos

        if (gamepad1.dpad_up) {
            sClawL.setPosition(sClawL.getPosition() - 0.05);
            sClawR.setPosition(sClawR.getPosition() - 0.05);
            sleep(300);
        }
        else if (gamepad1.dpad_down) {
            sClawL.setPosition(sClawL.getPosition() + 0.05);
            sClawR.setPosition(sClawR.getPosition() + 0.05);
            sleep(300);
        }
        telemetry.addData("sClaw values", "Left %f, Right %f",
                sClawL.getPosition(), sClawR.getPosition()); */
    }
}