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
@Disabled
@Autonomous(name="Blue Loading (Good Al.)", group="Linear Opmode") // assuming our teammate's robot will move the foundation to the building site
public class LoadingZoneAuto5361 extends LinearOpMode {
    // Declare OpMode members.
    // public boolean isBlueAlliance = true; //Set to false if red alliance
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorBL, motorBR, motorFL, motorFR, strafeMotor, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL, bClawM; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left/Middle

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
		
        motorFL.setPower(.5);
		motorBL.setPower(.5);
        motorFR.setPower(.5);
		motorBR.setPower(.5);
		
        sleep(1000);

        telemetry.addData("Status", "Grabbing stone #1");	
        telemetry.update();
		
        motorFL.setPower(0);
		motorBL.setPower(0);
		motorFR.setPower(0);
		motorBR.setPower(0);
		
        sleep(800);
		
        servoFL.setPosition(.34);
        servoFR.setPosition(.47);
		
        sleep(500);
		
        clawUpDown.setPosition(.7);
        sleep(800);

        telemetry.addData("Status","Turning back");
        telemetry.update();
		
        motorFL.setPower(-1);
		motorBL.setPower(-1);
		motorFR.setPower(.1);
		motorBR.setPower(.1);
		
        sleep(900);

        telemetry.addData("Status", "Crossing bridge");
        telemetry.update();
		
        motorFL.setPower(0.7);
		motorBL.setPower(0.7);
		motorFR.setPower(0.7);
		motorBR.setPower(0.7);
		
        sleep(1250);

        telemetry.addData("Status", "Release stone #1");
        telemetry.update();
		
        motorFL.setPower(0);
        motorBL.setPower(0);
		motorFR.setPower(0);
		motorBR.setPower(0);
		
        clawUpDown.setPosition(.1);
		
        sleep(300);
		
        servoFL.setPosition(.22);
        servoFR.setPosition(.30);
		
        sleep(500);

        telemetry.addData("Status","Back to Loading Zone for stone #2");
        telemetry.update();
		
        motorFL.setPower(-0.7);
        motorBL.setPower(-0.7);
		motorFR.setPower(-0.7);
		motorBR.setPower(-0.7);
		
        sleep(2300);  // 2300 if going back for a second block
		
        motorFL.setPower(0);
        motorBL.setPower(0);
		motorFR.setPower(0);
		motorBR.setPower(0);
		
        sleep(300);

        telemetry.addData("Status", "Turn to face for 2nd Block");
        telemetry.update();
		
        motorFL.setPower(1);
		motorBL.setPower(1);
        motorFR.setPower(-.1);
		motorBR.setPower(-.1);
		
        sleep(1100);

        telemetry.addData("Status", "Grabbing stone #2");
        telemetry.update();
		
        motorFL.setPower(0);
        motorBL.setPower(0);
		motorFR.setPower(0);
		motorBR.setPower(0);
		
        sleep(800);
		
        servoFL.setPosition(.34);
        servoFR.setPosition(.47);
		
        sleep(500);
		
        clawUpDown.setPosition(.7);
        sleep(800);

        telemetry.addData("Status","Turning back");
        telemetry.update();
		
        motorFL.setPower(-1);
        motorBL.setPower(-1);
		motorFR.setPower(.1);
		motorBR.setPower(.1);
		
        sleep(1050);

        telemetry.addData("Status", "Crossing bridge");
        telemetry.update();
		
        motorFL.setPower(0.7);
		motorBL.setPower(0.7)
        motorFR.setPower(0.7);
		motorBR.setPower(0.7);
		
        sleep(1250);

        telemetry.addData("Status", "Release stone #1");
        telemetry.update();
		
        motorFL.setPower(0);
		motorBL.setPower(0);
		motorFR.setPower(0);
        motorBR.setPower(0);
		
        clawUpDown.setPosition(.1);
		
        sleep(300);
		
        servoFL.setPosition(.22);
        servoFR.setPosition(.30);
		
        sleep(500);

        telemetry.addData("Status", "Parking under Bridge");
        telemetry.update();
		
        motorFL.setPower(.5);
		motorBL.setPower(.5);
		motorFR.setPower(.5);
        motorBR.setPower(.5);
		
        sleep(500);
		
        motorFL.setPower(0);
		motorBL.setPower(0);
		motorFR.setPower(0);
        motorBR.setPower(0);

        telemetry.addData("Dinner", "Served <0/");
        telemetry.update(); 
    }

    private void setUp(){ //account for alliance
        telemetry.addData("Status", "Initialized");
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

            motorBL.setDirection(DcMotor.Direction.REVERSE);
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
            strafeMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            motorBR = hardwareMap.dcMotor.get("motorBL");
            motorFR = hardwareMap.dcMotor.get("motorFL");
            motorBL = hardwareMap.dcMotor.get("motorBR");
            motorFL = hardwareMap.dcMotor.get("motorFR");
            strafeMotor = hardwareMap.dcMotor.get("motorM");

            motorBR.setDirection(DcMotor.Direction.REVERSE);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            strafeMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        clawTower = hardwareMap.dcMotor.get("clawTower");
        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");
        bClawM = hardwareMap.servo.get("blockClawM");

        //switch these if they are going backward
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);
        bClawM.setDirection(Servo.Direction.REVERSE);
    }
    /* //Account for the 5 driving motors
    public void encoderDrive(double speed,
                             double FLin, double FRin, double BLin, double BRin,
                             double timeoutS) { // middle inputs are how many inches to travel
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = FR.getCurrentPosition() + (int) (FLin * COUNTS_PER_INCH);
            newFRTarget = FL.getCurrentPosition() + (int) (FRin * COUNTS_PER_INCH);
            newBLTarget = BL.getCurrentPosition() + (int) (BLin * COUNTS_PER_INCH);
            newBRTarget = BR.getCurrentPosition() + (int) (BRin * COUNTS_PER_INCH);

            FR.setTargetPosition(newFLTarget);
            FL.setTargetPosition(newFRTarget);
            BL.setTargetPosition(newBLTarget);
            BR.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FR.isBusy() && FL.isBusy() && BL.isBusy() && BR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d", newFLTarget, newFRTarget, newBLTarget, newFLTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d",
                        FR.getCurrentPosition(),
                        FL.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Turn off RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    } */
}
