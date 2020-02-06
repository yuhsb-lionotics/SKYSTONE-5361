package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Stones [TEST]", group="Linear Opmode")
public class ColorLoadingZoneAuto5361 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorBL, motorBR, motorFL, motorFR, strafeMotor, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left/Middle
    private ColorSensor bridgeColor, wallColor;

    public boolean getIsBlueAlliance() {return true;} //Set to false if red alliance

    private static final double COUNTS_PER_MOTOR_REV = 1220;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        setUp();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
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

        telemetry.addData("Status", "Grabbing stone #1");
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
        sleep(900);

        telemetry.addData("Status", "Crossing bridge");
        telemetry.update();
        leftMotor.setPower(0.7);
        rightMotor.setPower(0.7);
        sleep(1250);

        telemetry.addData("Status", "Release stone #1");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        clawUpDown.setPosition(.1);
        sleep(300);
        servoFL.setPosition(.22);
        servoFR.setPosition(.30);
        sleep(500);

        telemetry.addData("Status","Back to Loading Zone for stone #2");
        telemetry.update();
        leftMotor.setPower(-0.7);
        rightMotor.setPower(-0.7);
        sleep(2300);  // 2300 if going back for a second block
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(300);

        telemetry.addData("Status", "Turn to face for 2nd Block");
        telemetry.update();
        leftMotor.setPower(1);
        rightMotor.setPower(-.1);
        sleep(1100);

        telemetry.addData("Status", "Grabbing stone #2");
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
        sleep(1250);

        telemetry.addData("Status", "Release stone #1");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        clawUpDown.setPosition(.1);
        sleep(300);
        servoFL.setPosition(.22);
        servoFR.setPosition(.30);
        sleep(500);

        telemetry.addData("Status", "Parking under Bridge");
        telemetry.update();
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(500);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        telemetry.addData("Dinner", "Served <0/");
        telemetry.update(); */ //<--old code

        // run until the end of the match (driver presses STOP)
        sClawL.setPosition(1);
        sClawR.setPosition(1);
        //set stone claw all the way closed

        encoderDrive(0.7, 25, 25, 0, 3);          //towards stones

        String skystonePosition = detectSkystone();
        if (skystonePosition == "Center") {
            telemetry.addData("Block Pos:", "Center");
            telemetry.update();
            encoderDrive(.3, -3, -3, 0, 1.5);
            sleep(500);
        }
        if (skystonePosition == "Bridge") {
            telemetry.addData("Block Pos:", "Bridge");
            telemetry.update();
            encoderDrive(.3, -3, -3, 0, 1.5);
            encoderDrive(.3, 0, 0, 2, 1.0);
            sleep(500);
        }
        if (skystonePosition == "Wall")   {
            telemetry.addData("Block Pos:", "Wall");
            telemetry.update();
            encoderDrive(.3, -3, -3, 0, 1.5);
            encoderDrive(.3, 0, 0, -2, 1.0);
            sleep(500);
        }

        sClawL.setPosition(.4); sClawR.setPosition(.5);                                     //open claw
        encoderDrive(.5, 14, 14, 0, 2.0);      //towards block
        sClawL.setPosition(.8); sClawR.setPosition(.9);                                      //grab block
        encoderDrive(.5, -15, -15, 0, 2.0);     //move back
        encoderDrive(.7, 0, 0, 36, 3.0);        //cross bridge
        sClawL.setPosition(.2); sClawR.setPosition(.1);                                     //open claw (wide to let block slide out of claw)
    }
            //NOTE --- the 6 measurements above are programed for "Wall", so it will need to be put into
            // skystonePos==Wall and we will need to copy the 6 lines above and paste it into ==Bridge and ==Center

    private void setUp(){ //account for alliance
        telemetry.addData("Status", "Initialized - Setting Up");
        telemetry.update();

        // Exchange right and left if in the red alliance.
        // We may need to assign a different variable to represent motors that don't switch for vuforia,
        // if the camera is on the side of the phone.
        if(getIsBlueAlliance()){
            motorBL = hardwareMap.dcMotor.get("motorBL");
            motorFL = hardwareMap.dcMotor.get("motorFL");
            motorBR = hardwareMap.dcMotor.get("motorBR");
            motorFR = hardwareMap.dcMotor.get("motorFR");
            strafeMotor = hardwareMap.dcMotor.get("motorM");
            bridgeColor = hardwareMap.colorSensor.get("colorL");
            wallColor = hardwareMap.colorSensor.get("colorR");

            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            strafeMotor.setDirection(DcMotor.Direction.FORWARD); //Positive values go to the left
        } else {
            motorBR = hardwareMap.dcMotor.get("motorBL");
            motorFR = hardwareMap.dcMotor.get("motorFL");
            motorBL = hardwareMap.dcMotor.get("motorBR");
            motorFL = hardwareMap.dcMotor.get("motorFR");
            strafeMotor = hardwareMap.dcMotor.get("motorM");
            bridgeColor = hardwareMap.colorSensor.get("colorR");
            wallColor = hardwareMap.colorSensor.get("colorL");

            motorBR.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            strafeMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        clawTower = hardwareMap.dcMotor.get("clawTower");
        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");

        //switch these if they are going backward
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);


        //resetting encoders & waiting
        telemetry.addData("Status", "Resetting Encoder");
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

//start loading auto
        //sClawL.setPosition(0.6); //open perfect
        //sClawR.setPosition(0.75);
        sClawL.setPosition(1);
        sClawR.setPosition(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private String detectSkystone() {
        String skystonePosition;
        telemetry.addData("Left sensor (RGBHV):", "%d, %d, %d, %d, %d",
                bridgeColor.red(), bridgeColor.green(), bridgeColor.blue(), bridgeColor.argb(), bridgeColor.alpha());
        telemetry.addData("Right sensor (RGBHV):", "%d, %d, %d, %d, %d",
                wallColor.red(), wallColor.green(), wallColor.blue(), wallColor.argb(), wallColor.alpha());
        if (bridgeColor.argb() == 0) {skystonePosition = "Bridge";}
        else if (wallColor.argb() == 0) {skystonePosition = "Wall";}
        //The left and right sensors should not be compared to each other, since even a small differential in distance changes it a lot
        else if (bridgeColor.red() + bridgeColor.green() < bridgeColor.blue() * 3.5) {
            skystonePosition = "Bridge";
        } else if (wallColor.red() + wallColor.green() < wallColor.blue() * 3.5) {
            skystonePosition = "Wall";
        } else {skystonePosition = "Center";}
        telemetry.addData("Skystone", skystonePosition);
        telemetry.update();
        return skystonePosition;
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double strafeInches,
                             double timeoutS) { // middle inputs are how many inches to travel
        int newFRTarget;
        int newFLTarget;
        int newBLTarget;
        int newBRTarget;
		int newSMTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFRTarget = motorFR.getCurrentPosition()     + (int) (leftInches   * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition()     + (int) (rightInches  * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition()     + (int) (leftInches   * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition()     + (int) (rightInches  * COUNTS_PER_INCH);
			newSMTarget = strafeMotor.getCurrentPosition() + (int) (strafeInches * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBL.setTargetPosition(newBLTarget);
            motorBR.setTargetPosition(newBRTarget);
			strafeMotor.setTargetPosition(newSMTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			strafeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
			strafeMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((motorFR.isBusy() && motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy()) || strafeMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d :%7d",
                        newFRTarget, newFLTarget, newBLTarget, newFRTarget, newSMTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d :%7d",
                        motorFR.getCurrentPosition(),
                        motorFL.getCurrentPosition(),
                        motorBL.getCurrentPosition(),
                        motorBR.getCurrentPosition(),
						strafeMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
			strafeMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			strafeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
