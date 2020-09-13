package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Club Fair Autonomous", group="Linear Opmode")
public class ClubFairAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorBL, motorBR, motorFL, motorFR, strafeMotor, clawTower, dude111;
    private Servo sClawR, sClawL, fGripR, fGripL, compressor; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left/Middle
    private ColorSensor leftColor, rightColor;

    private final double SKYSTONE_DISTANCE = 18; //inches
    private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
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

        //set stone claw all the way closed
        TeleOp5361.closeClaw(sClawL, sClawR);
        //unsquish and move servos into position
        compressor.setPosition(.98);//change if compressor malfunctions
        sleep(300);
        clawTower.setPower(-.7);
        sleep(100);
        clawTower.setPower(0);

        encoderDrive(0.3, 3, 3, 0, 5);              //slow at start - it shakes if it starts fast against the wall
        encoderDrive(0.7, SKYSTONE_DISTANCE-3, SKYSTONE_DISTANCE-3, 0, 5);    //towards stones
        encoderDrive(.7, 0, 0, 5.5, 5);            //moving to 5th block
        String skystonePosition = detectSkystone();
        encoderDrive(.7, -3, -3, 0.0, 5);         //back up

        char skyStonePos = 'Q'; //may be unnecessary but can't hurt

        if (skystonePosition == "Center") {
            skyStonePos = 'C';
            telemetry.addData("Block Pos:", "Center");
            telemetry.update();
            sleep(250);
        }
        if (skystonePosition == "Bridge") {
            skyStonePos = 'B';
            telemetry.addData("Block Pos:", "Bridge");
            telemetry.update();
            encoderDrive(.7, 0, 0, 9.5, 5);
            sleep(250);
        }
        if (skystonePosition == "Wall")   {
            skyStonePos = 'W';
            telemetry.addData("Block Pos:", "Wall");
            telemetry.update();
            encoderDrive(.7, 0, 0, -9.5, 5);//[.2,1,1 sometimes overshoots 1/2 block]get marmon to fix motorM
            sleep(250);
        }

        //move#0: grasping stone#1
        TeleOp5361.openClaw(sClawL, sClawR);
        sleep(100);
        encoderDrive(.4, 7, 7, 0, 5);
        TeleOp5361.grabStone(sClawL, sClawR);
        sleep(100);
        clawTower.setPower(.7);
        sleep(400);
        clawTower.setPower(0);
        sleep(1000);

    }


    private void setUp(){ //account for alliance
        telemetry.addData("Status", "Initialized - Setting Up");
        telemetry.update();

        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        strafeMotor = hardwareMap.dcMotor.get("motorM");
        leftColor = hardwareMap.colorSensor.get("colorL");
        rightColor = hardwareMap.colorSensor.get("colorR");

        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        strafeMotor.setDirection(DcMotor.Direction.FORWARD); //Positive values go to the left

        clawTower = hardwareMap.dcMotor.get("clawTower");
        dude111 = hardwareMap.dcMotor.get("tapeTongue");
        sClawL = hardwareMap.servo.get("blockClawL");
        sClawR = hardwareMap.servo.get("blockClawR");
        fGripL = hardwareMap.servo.get("foundationGripL");
        fGripR = hardwareMap.servo.get("foundationGripR");
        compressor = hardwareMap.servo.get("compressor");

        //switch these if they are going backward
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        dude111.setDirection(DcMotor.Direction.FORWARD);
        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);
        compressor.setDirection(Servo.Direction.FORWARD);


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
        fGripL.setPosition(0.19);
        fGripR.setPosition(0.19);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private String detectSkystone() {
        String skystonePosition;
        telemetry.addData("Left Yellowness ratio",
                (leftColor.red() + leftColor.green()) / (double) leftColor.blue());
        telemetry.addData("Right Yellowness ratio",
                (rightColor.red() + rightColor.green()) / (double) rightColor.blue());
        if (leftColor.argb() == 0) {skystonePosition = "Left";}
        else if (rightColor.argb() == 0) {skystonePosition = "Right";}
        //The left and right sensors should not be compared to each other, since even a small differential in distance changes it a lot
        else if (leftColor.red() + leftColor.green() < leftColor.blue() * 3.3) {
            skystonePosition = "Left";
        } else if (rightColor.red() + rightColor.green() < rightColor.blue() * 3.3) {
            skystonePosition = "Right";
        } else {skystonePosition = "Center";}
        telemetry.addData("Skystone", skystonePosition);
        telemetry.update();
        return skystonePosition;
    }

    protected void encoderDrive(double speed,
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
            newFRTarget = motorFR.getCurrentPosition()     + (int) (rightInches   * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition()     + (int) (leftInches  * COUNTS_PER_INCH);
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
                    ((motorFR.isBusy() && motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy()) || (strafeMotor.isBusy()))) {

                // Display it for the driver.
                /* telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d :%7d",
                        newFRTarget, newFLTarget, newBLTarget, newFRTarget, newSMTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d :%7d",
                        motorFR.getCurrentPosition(),
                        motorFL.getCurrentPosition(),
                        motorBL.getCurrentPosition(),
                        motorBR.getCurrentPosition(),
						strafeMotor.getCurrentPosition()); */
            }

            telemetry.addData("Encoder Drive", "Finished in %.2f s/%f", runtime.seconds(), timeoutS);
            telemetry.update();

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
