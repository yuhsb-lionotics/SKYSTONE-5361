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
    private DcMotor motorFL, motorFR, motorBL, motorBR, strafeMotor, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left
    private ColorSensor leftColor, rightColor;
    private String driveMode = "Tank Control"; //Values are "Tank Control" and "Joystick Control".
                                               //Press Y on the controller to change the mode.
    //private boolean yWasPressed = false;
    private String driverMode = "Power"; //shifts between "Power" and "Precision" motor controls


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
            telemetry.addData("Left sensor (RGBHV):", "%d, %d, %d, %d, %d",
                    leftColor.red(), leftColor.green(), leftColor.blue(), leftColor.argb(), leftColor.alpha());
            telemetry.addData("Right sensor (RGBHV):", "%d, %d, %d, %d, %d",
                    rightColor.red(), rightColor.green(), rightColor.blue(), rightColor.argb(), rightColor.alpha());
            if (leftColor.argb() == 0) {telemetry.addData("Skystone", "Left");}
            else if (rightColor.argb() == 0) {telemetry.addData("Skystone", "Right");}
            //The left and right sensors should not be compared to each other, since even a small differential in distance changes it a lot
            else if (leftColor.red() + leftColor.green() < leftColor.blue() * 3.5) {
                telemetry.addData("Skystone", "Left");
            } else if (rightColor.red() + rightColor.green() < rightColor.blue() * 3.5) {
                telemetry.addData("Skystone", "Right");
            } else {telemetry.addData("Skystone", "Center");}
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
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
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
    private void driveCalc() //turns the gamepad controls into omniwheel commands
    {
        double leftPower; double rightPower; double leftPrecision; double rightPrecision;

        /* if (!yWasPressed & gamepad1.y) {
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
        else {
            telemetry.addData("Error", "Unacceptable Drive Mode");
            leftPower = 0;
            rightPower = 0;
            leftPrecision = 0;
            rightPrecision = 0;
        }


    // write the values to the motors
        //BETA Controls (Precision)
        if(driverMode == "Precision") {
            telemetry.addData("Drive Control:", "Precision-BETA");

            if (gamepad2.left_bumper && gamepad2.right_bumper)
                { driverMode = "Power"; } //BETA give control to ALPHA (Power)
            motorBL.setPower(leftPrecision);
            motorFL.setPower(leftPrecision);  //joysticks power
            motorBR.setPower(rightPrecision);
            motorFR.setPower(rightPrecision);
            strafeMotor.setPower((gamepad2.left_trigger - gamepad2.right_trigger) / 4); //strafe
            /*if(gamepad2.left_trigger >= 0)    { strafeMotor.setPower(leftPrecision); }
            if(gamepad2.right_trigger >= 0)     { strafeMotor.setPower(-leftPrecision); }*/
        }
        //ALPHA Controls (Drive)
        else if (driverMode == "Power") {
            telemetry.addData("Drive Control:", "Power-ALPHA");
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                driverMode = "Precision";
            } //ALPHA give control to BETA (Precision)
            motorBL.setPower(leftPower);
            motorFL.setPower(leftPower); //joysticks power
            motorBR.setPower(rightPower);
            motorFR.setPower(rightPower);
            strafeMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger); //strafe
            /*if(gamepad1.left_trigger >= 0)    { strafeMotor.setPower(leftPower); }
            if(gamepad1.right_trigger >= 0)     { strafeMotor.setPower(-leftPower); }*/
        }

        String teleFormat = "leftPower (%.2f), rightPower (%.2f), leftPrecision (%.2f), rightPrecision (%.2f)";
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", teleFormat, leftPower, rightPower, leftPrecision, rightPrecision);
    }

    private void roboCalc() {
        //foundation Grab //works
        if (gamepad2.dpad_up)   {fGripL.setPosition(.13); fGripR.setPosition(.1);} //up
        else if (gamepad2.dpad_down) {fGripL.setPosition(.78); fGripR.setPosition(.75);} //down
        else if (gamepad1.dpad_up)   {fGripL.setPosition(.13); fGripR.setPosition(.1);} //up
        else if (gamepad1.dpad_down) {fGripL.setPosition(.78); fGripR.setPosition(.75);} //down

        //tower lift //works
        if (gamepad2.y)         {clawTower.setPower(1); sleep(300); clawTower.setPower(0);} //tower up
        else if (gamepad2.a)    {clawTower.setPower(-1); sleep(300); clawTower.setPower(0);} //tower down
        else if (gamepad1.y)    {clawTower.setPower(1); sleep(300); clawTower.setPower(0);} //tower up
        else if (gamepad1.a)    {clawTower.setPower(-1); sleep(300); clawTower.setPower(0);} //tower down

        //stone grab //works
        if (gamepad1.x)         {sClawL.setPosition(.05); sClawR.setPosition(.12);} //open - originally both .1 //consider making larger
        else if (gamepad1.b)    {sClawL.setPosition(.34); sClawR.setPosition(.47);} // close - originally both .4
        else if (gamepad2.x)    {sClawL.setPosition(.05); sClawR.setPosition(.12);} //open - originally both .1 //consider making larger
        else if (gamepad2.b)    {sClawL.setPosition(.34); sClawR.setPosition(.47);} // close - originally both .4

        /*debugging
        if (gamepad1.dpad_left) {
            bClawM.setPosition(gamepad1.right_trigger/2);
            //telemetry.addData("Claw Position", gamepad1.right_trigger/10);
        }telemetry.addData("Claw Position", bClawM.getPosition()); */
    }
}