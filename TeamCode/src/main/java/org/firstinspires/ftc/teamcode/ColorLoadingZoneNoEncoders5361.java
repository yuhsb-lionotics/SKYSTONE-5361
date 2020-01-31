package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Loading No Encoders", group="Linear Opmode")
public class ColorLoadingZoneNoEncoders5361 extends LinearOpMode {
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

        // run until the end of the match (driver presses STOP)
        telemetry.addData("Status","Toward stone");
        telemetry.update();
        fGripL.setPosition(0.13);
        fGripR.setPosition(0.1);
        //set stone claw all the way closed
        sClawL.setPosition(.16); //may need adjusting
        sClawR.setPosition(.25);
        setMotors(.5, .5, 0);
        sleep(1000);
        setMotors(0, 0, 0);
        sleep(800);

        String skystonePosition = detectSkystone();
        setMotors(-.5, -.5, 0);
        sleep(350);
        setMotors(0,0,0);
        sleep(300);
        if (skystonePosition == "Center") {
            telemetry.addData("Status", "Taking center stone");
            telemetry.update();
            sClawL.setPosition(.05); sClawR.setPosition(.12);
            sleep(500);
        }
        if (skystonePosition == "Bridge") {
            telemetry.addData("Status", "Taking bridge stone");
            telemetry.update();
            setMotors(0, 0, .5);
            sleep(235);
            sClawL.setPosition(.05); sClawR.setPosition(.12);
            sleep(500);
        }
        if (skystonePosition == "Wall")   {
            telemetry.addData("Status", "Taking wall stone");
            telemetry.update();
            setMotors(0, 0, -.5);
            sleep(235);
            sClawL.setPosition(.05); sClawR.setPosition(.12);
            sleep(500);
        }
        /*
        Old Code:
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
        telemetry.update(); */
    }

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
            motorFL.setDirection(DcMotor.Direction.REVERSE);
            motorBR.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.FORWARD);
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

        //switch these if they are going backward
        clawTower.setDirection(DcMotor.Direction.FORWARD);
        sClawL.setDirection(Servo.Direction.FORWARD);
        sClawR.setDirection(Servo.Direction.REVERSE);
        fGripL.setDirection(Servo.Direction.REVERSE);
        fGripR.setDirection(Servo.Direction.FORWARD);


        //Turn off encoders
        motorFR.setMode    (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode    (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode    (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode    (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized and Set Up");
        telemetry.update();
    }

    private String detectSkystone() {
        String skystonePosition;
        telemetry.addData("Status", "Detecting Skystone");
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

    private void setMotors(double leftPower, double rightPower, double strafePower) { //set power to driving motors
        motorFL.setPower(leftPower);
        motorBL.setPower(leftPower);
        motorFR.setPower(rightPower);
        motorBR.setPower(rightPower);
        strafeMotor.setPower(strafePower);
    }
}
