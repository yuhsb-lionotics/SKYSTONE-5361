package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CONTROLS:
 * There is one driver.
 * Joysticks to drive (tank control)
 * Y/A to raise and lower claw tower
 * X/B to toggle stone claw
 * DPad up/down to toggle foundation grip
 * Dpad left/right to extend/retract tape measure
 * Triggers - strafe
 * Bumpers - toggle capstone arm
 *
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

@TeleOp(name="Club Fair TeleOp", group="Linear Opmode")
public class ClubFairTeleOp extends LinearOpMode {

    // Declare OpMode members. //RECODING FOR 6 MOTORS, 4 SERVOS
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL, motorFR, motorBL, motorBR, strafeMotor, clawTower, tapeMeasure;
    private Servo sClawR, sClawL, fGripR, fGripL, compressor; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left
    private ColorSensor leftColor, rightColor;
    private Servo capstoneArm; //capstone arm
    private boolean shakeToggle = true;


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

            if(gamepad1.right_bumper) {
                //shake the capstone arm
                while(gamepad1.right_bumper);
                if (shakeToggle) {
                    capstoneArm.setPosition(.35);
                    shakeToggle = false;
                } else{
                    capstoneArm.setPosition(.30);
                    shakeToggle = true;
                }
            }
            if(gamepad1.left_bumper){
                capstoneArm.setPosition(.9);
            }
        }
    }
    public static void openClaw(Servo leftServo, Servo rightServo) {
        leftServo.setPosition(.4);
        rightServo.setPosition(.35);
    }
    public static void grabStone(Servo leftServo, Servo rightServo) {
        leftServo.setPosition(.9);
        rightServo.setPosition(.9);
    }
    public static void closeClaw(Servo leftServo, Servo rightServo) { // all the way
        leftServo.setPosition(1);
        rightServo.setPosition(.9);
    }
    public static void wideClaw(Servo leftServo, Servo rightServo){
        leftServo.setPosition(.15);
        rightServo.setPosition(.05);
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
        tapeMeasure = hardwareMap.dcMotor.get("tapeTongue");


        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");
        capstoneArm = hardwareMap.servo.get("capstoneArm");
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
        tapeMeasure.setDirection(DcMotor.Direction.FORWARD);

        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);
        capstoneArm.setDirection(Servo.Direction.FORWARD);
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

        //Assign values to leftPower and rightPower
        leftPower       =   (-gamepad1.left_stick_y)*.5;
        rightPower      =   (-gamepad1.right_stick_y)*.5;

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

        if      (gamepad1.dpad_up)   {fGripL.setPosition(.19); fGripR.setPosition(.19);} //up
        else if (gamepad1.dpad_down) {fGripL.setPosition(.78); fGripR.setPosition(.78);} //down

        if      (gamepad1.y)  {clawTower.setPower(.7); } //tower up
        else if (gamepad1.a)  {clawTower.setPower(-.7); } //tower down
        else                  {clawTower.setPower(0);} //tower idle

        if      (gamepad1.dpad_left)  {
            tapeMeasure.setPower(.7);}
        else if (gamepad1.dpad_right) {
            tapeMeasure.setPower (-.7);}
        else {tapeMeasure.setPower(0);}
    }
}