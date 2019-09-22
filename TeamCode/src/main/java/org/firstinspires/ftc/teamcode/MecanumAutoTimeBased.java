package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/* This is an autonomous code based on the setup we had last year. */

@Autonomous(name="Time-Based Auto", group="Autonomous")
public class MecanumAutoTimeBased extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL, motorBL, motorFR, motorBR, landerRiser;
    private Servo markerDrop;


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        landerRiser = hardwareMap.get(DcMotor.class, "lander riser");
        landerRiser = hardwareMap.get(DcMotor.class, "lander riser");
        landerRiser.setDirection(DcMotor.Direction.FORWARD);
        landerRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        markerDrop = hardwareMap.get(Servo.class, "marker");
        markerDrop.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Put drive commands here, add in telemetry.
    }

    private void move(double power, int millis, boolean stopAfter) { strafe(power, power, millis, stopAfter); }

    private void strafeLeft(double power, int millis, boolean stopAfter) { strafe(-power, power, millis, stopAfter); }

    //powerA goes forward and to the right, powerB goes forward and to the left, where forward is the front according to the config (not real life).
    private void strafe(double powerA, double powerB, int millis, boolean stopAfter) {
        motorFL.setPower(-powerA);
        motorBL.setPower(-powerB);
        motorFR.setPower( powerB);
        motorBR.setPower( powerA);

        sleep(millis);

        if (stopAfter) {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
            sleep(200);
        }
    }

    private void rotateLeft(double power, int millis, boolean stopAfter) {
        motorFL.setPower( power);
        motorBL.setPower( power);
        motorFR.setPower( power);
        motorBR.setPower( power);

        sleep(millis);

        if (stopAfter) {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
            sleep(200);
        }
    }
}