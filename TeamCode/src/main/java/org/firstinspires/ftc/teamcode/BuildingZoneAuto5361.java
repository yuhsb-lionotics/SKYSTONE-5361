package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Foundation [TEST]", group="Linear Opmode")
public class BuildingZoneAuto5361 extends LinearOpMode {
    // Declare OpMode members.
    // public boolean isBlueAlliance = true; //Set to false if red alliance
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorBL, motorBR, motorFL, motorFR, strafeMotor, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL, compressor; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left/Middle

    public boolean getIsBlueAlliance() {return true;}

    private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void runOpMode() {
        setUp();
        waitForStart();
        //runtime.reset();
/*
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
        */  //<--old code (no encoders)

        encoderDrive(0.7, -36, -36, 0, 4); // move towards foundation (fClaw facing foundation)
        fGripL.setPosition(.78);
        fGripR.setPosition(.78); //grab onto Foundation
        sleep(500);
        encoderDrive(0.7, 44, 44, 0, 5); // move towards foundation (fClaw facing foundation)
        fGripL.setPosition(.78);
        fGripR.setPosition(.78); //grab onto Foundation
        sleep(400);
        encoderDrive(0.7, -12, 12, 8, 3); //turning Foundation
        fGripL.setPosition(.25);
        fGripR.setPosition(.22); //unlatch Foundation
        encoderDrive(0.7, 24, -24, -2, 4);

    }
    private void setUp(){ //account for alliance
        telemetry.addData("Status", "Resetting Encoder");
        telemetry.update();
        if(getIsBlueAlliance()){
            motorBL = hardwareMap.dcMotor.get("motorBL");
            motorFL = hardwareMap.dcMotor.get("motorFL");
            motorBR = hardwareMap.dcMotor.get("motorBR");
            motorFR = hardwareMap.dcMotor.get("motorFR");
            strafeMotor = hardwareMap.dcMotor.get("motorM");

            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            strafeMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            motorBR = hardwareMap.dcMotor.get("motorBL");
            motorFR = hardwareMap.dcMotor.get("motorFL");
            motorBL = hardwareMap.dcMotor.get("motorBR");
            motorFL = hardwareMap.dcMotor.get("motorFR");
            strafeMotor = hardwareMap.dcMotor.get("motorM");

            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.REVERSE);
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            strafeMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        clawTower = hardwareMap.dcMotor.get("clawTower");
        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");
        compressor = hardwareMap.servo.get("compressor");


        //switch these if they are going backward
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);
        compressor.setDirection(Servo.Direction.FORWARD);

        //resetting encoders & waiting
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

        fGripL.setPosition(0.19);
        fGripR.setPosition(0.19);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
            newSMTarget = strafeMotor.getCurrentPosition() + (int) (strafeInches * COUNTS_PER_INCH / 1.5); //gear reduction

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
                    (motorFR.isBusy() && motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() || strafeMotor.isBusy())) {

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
