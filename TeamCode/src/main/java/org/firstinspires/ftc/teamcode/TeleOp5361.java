package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotor motorFL, motorFR, motorBL, motorBR, strafeMotor, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left
    private ColorSensor leftColor, rightColor;
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
        while (opModeIsActive()) {
            omniCalc();
            telemetry.addData("Left sensor (RGBV):", "%d, %d, %d, %d",
                    leftColor.red(), leftColor.green(), leftColor.blue(), leftColor.alpha());
            telemetry.addData("Right sensor (RGBV):", "%d, %d, %d, %d",
                    rightColor.red(), rightColor.green(), rightColor.blue(), rightColor.alpha());
            telemetry.update();
            //idle();
        }
    }

    private void setUp(){ //account for alliance
        telemetry.addData("Status", "Initialized - Setting Up");
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
        leftColor = hardwareMap.colorSensor.get("colorL");
        rightColor = hardwareMap.colorSensor.get("colorR");

        //switch these if they are going backward
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        strafeMotor.setDirection(DcMotor.Direction.FORWARD);
        clawTower.setDirection(DcMotor.Direction.FORWARD);

        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);


        //resetting encoders & waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    private void omniCalc() //turns the gamepad controls into omniwheel commands
    {
        double leftPower; double rightPower; double leftPrecision; double rightPrecision;

     /*   if (!yWasPressed & gamepad1.y) {
            if (driveMode == "Tank Control") {driveMode = "Joystick Control";}
            else if (driveMode == "Joystick Control") {driveMode = "Tank Control";}
            else {telemetry.addData("Error", "Unacceptable driveMode");}
        }
        yWasPressed = gamepad1.y; */

        telemetry.addData("Drive Mode", driveMode);
        //Assign values to leftPower and rightPower
        if (driveMode == "Tank Control") {
            leftPower       =   -gamepad1.left_stick_y;
            rightPower      =   -gamepad1.right_stick_y;
            leftPrecision   =   (-gamepad2.left_stick_y)/4;
            rightPrecision  =   (-gamepad2.right_stick_y)/4;
        }
     /*   else if (driveMode == "Joystick Control") {
            leftPower = Range.clip(-gamepad1.right_stick_y + gamepad1.right_stick_x, -1, 1);
            rightPower = Range.clip(-gamepad1.right_stick_y - gamepad2.right_stick_x, -1, 1);
        } */
        else {telemetry.addData("Error", "Unacceptable Drive Mode"); telemetry.update(); leftPower = 0; rightPower = 0; leftPrecision = 0; rightPrecision = 0;}


    // write the values to the motors

        //driver B can give drive to driver A (Driver Power)
        if(gamepad2.left_bumper && gamepad2.right_bumper) { //BETA controller gives power to ALPHA
            //controls
            if(gamepad1.left_stick_y >= 0)      { motorBL.setPower(leftPower); motorFL.setPower(leftPower); }
            if(gamepad1.right_stick_y >= 0)     { motorBR.setPower(rightPower); motorFR.setPower(rightPower); }
            /*if(gamepad1.left_trigger >= 0)    { strafeMotor.setPower(leftPower); }
            if(gamepad1.right_trigger >= 0)     { strafeMotor.setPower(-leftPower); }*/
            if(gamepad1.left_trigger >= 0 || gamepad1.right_trigger >= 0)  { strafeMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger); }

            //anti-controls (disable controls for beta)
            if(gamepad2.left_stick_y >= 0)  { telemetry.addData("Error", "Stop it BETA"); telemetry.update(); }
            if(gamepad2.right_stick_y >= 0) { telemetry.addData("Error", "Stop it BETA"); telemetry.update(); }
            if(gamepad2.left_trigger >= 0)  { telemetry.addData("Error", "Stop it BETA"); telemetry.update(); }
            if(gamepad2.right_trigger >= 0) { telemetry.addData("Error", "Stop it BETA"); telemetry.update(); }
        }

        //driver A can give drive to driver B (Precision Power)
        if(gamepad1.left_bumper && gamepad1.right_bumper) { //ALPHA controller gives power to BETA
            //controls
            if(gamepad2.left_stick_y >= 0)      { motorBL.setPower(leftPrecision); motorFL.setPower(leftPrecision); }
            if(gamepad2.right_stick_y >= 0)     { motorBR.setPower(rightPrecision); motorFR.setPower(rightPrecision); }
            /*if(gamepad2.left_trigger >= 0)    { strafeMotor.setPower(leftPrecision); }
            if(gamepad2.right_trigger >= 0)     { strafeMotor.setPower(-leftPrecision); }*/
            if(gamepad2.left_trigger >= 0 || gamepad2.right_trigger >= 0)  { strafeMotor.setPower((gamepad2.left_trigger - gamepad2.right_trigger)/4); }

            //anti-controls (disable controls for alpha)
            if(gamepad1.left_stick_y >= 0)  { telemetry.addData("Error", "Stop it ALPHA"); telemetry.update(); }
            if(gamepad1.right_stick_y >= 0) { telemetry.addData("Error", "Stop it ALPHA"); telemetry.update(); }
            if(gamepad1.left_trigger >= 0)  { telemetry.addData("Error", "Stop it ALPHA"); telemetry.update(); }
            if(gamepad1.right_trigger >= 0) { telemetry.addData("Error", "Stop it ALPHA"); telemetry.update(); }
        }

        //foundation Grab //works
        if (gamepad2.dpad_up)   {fGripL.setPosition(.13); fGripR.setPosition(.1);} //up
        if (gamepad2.dpad_down) {fGripL.setPosition(.78); fGripR.setPosition(.75);} //down

        //tower lift //works
        if (gamepad2.y)         {clawTower.setPower(1); sleep(300); clawTower.setPower(0);} //tower up
        if (gamepad2.a)         {clawTower.setPower(-1); sleep(300); clawTower.setPower(0);} //tower down

        //stone grab //works
        if (gamepad1.x)         {sClawL.setPosition(.05); sClawR.setPosition(.12);} //open - originally both .1 //consider making larger
        if (gamepad1.b)         {sClawL.setPosition(.34); sClawR.setPosition(.47);} // close - originally both .4

        /*debugging
        if (gamepad1.dpad_left) {
            bClawM.setPosition(gamepad1.right_trigger/2);
            //telemetry.addData("Claw Position", gamepad1.right_trigger/10);
        }telemetry.addData("Claw Position", bClawM.getPosition()); */

        String teleFormat = "leftPower (%.2f), rightPower (%.2f), leftPrecision (%.2f), rightPrecision (%.2f)";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", teleFormat, leftPower, rightPower, leftPrecision, rightPrecision);
        telemetry.update();
    }
}