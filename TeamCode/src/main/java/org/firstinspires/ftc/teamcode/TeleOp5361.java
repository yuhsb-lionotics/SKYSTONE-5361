package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    // Declare OpMode members. //RECODING FOR 6 MOTORS, 5 SERVOS
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorBL, motorBR, motorFL, motorFR, strafeMotor, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL, bClawM; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left/Middle
    private String driveMode = "Tank Control"; //Values are "Tank Control" and "Joystick Control".
                                               //Press Y on the controller to change the mode.
    //private boolean yWasPressed = false;

    @Override
    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) { omniCalc();  /*idle();*/ }
    }

    private void setUp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        strafeMotor = hardwareMap.dcMotor.get("motorM");
        clawTower = hardwareMap.dcMotor.get("clawTower");
        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");
        bClawM = hardwareMap.servo.get("blockClawM");

        //switch these if the robot is going backward
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        strafeMotor.setDirection(DcMotor.Direction.FORWARD);
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);
        bClawM.setDirection(Servo.Direction.REVERSE);
    }
//thousand,,,work from here down, recode 6motors and 5servos (5wheels+1eightytwenty tower, 2foundation grips, 3 prongs to hold block)
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
        //Assign values to leftPower and rightPower
        if (driveMode == "Tank Control") {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

        }/* else if (driveMode == "Joystick Control") {
            leftPower = Range.clip(-gamepad1.right_stick_y + gamepad1.right_stick_x, -1, 1);
            rightPower = Range.clip(-gamepad1.right_stick_y - gamepad2.right_stick_x, -1, 1);
        } */ else {telemetry.addData("Error", "Unacceptable driveMode"); telemetry.update(); leftPower = 0; rightPower = 0;}

        // write the values to the motors
        motorBL.setPower(leftPower);
        motorBR.setPower(rightPower);

        if (gamepad1.right_bumper) {fGripL.setPosition(.13); fGripR.setPosition(.1);} //up
        if (gamepad1.left_bumper) {fGripL.setPosition(.78); fGripR.setPosition(.75);} //down
        if (gamepad1.b) {sClawL.setPosition(.05); sClawR.setPosition(.12);} //open - originally both .1
        if (gamepad1.x) {sClawL.setPosition(.34); sClawR.setPosition(.47);} // close - originally both .4
        if (gamepad1.y) {bClawM.setPosition(0.7);} //up
        if (gamepad1.a) {bClawM.setPosition(0.1);} //down

        /*
        float rTrigger = gamepad1.right_trigger;
        float lTrigger = gamepad1.left_trigger;
        if(rTrigger > 0){
            strafeMotor.setPower(-rTrigger);
        } else if(lTrigger > 0){
            strafeMotor.setPower(lTrigger);
        } */
        strafeMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        /* if (gamepad1.dpad_left) { //debugging
            bClawM.setPosition(gamepad1.right_trigger/2);
            //telemetry.addData("Claw Position", gamepad1.right_trigger/10);
        }telemetry.addData("Claw Position", bClawM.getPosition()); */

        //What does this do?
        String teleFormat = "leftPower (%.2f), rightPower (%.2f)";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", teleFormat, leftPower, rightPower);
        telemetry.update();
    }
}