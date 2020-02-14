package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Extend Tape Auto", group="Linear Opmode")
public class ExtendTape5361 extends LinearOpMode {
    // Declare OpMode members.
    // public boolean isBlueAlliance = true; //Set to false if red alliance
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tapeExtender, clawTower;
    private Servo sClawR, sClawL, fGripR, fGripL, compressor; //fGrip : foundationGripRight/Left, sClaw : stoneClawRight/Left/Middle

    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        tapeExtender = hardwareMap.dcMotor.get("tapeTongue");
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

        TeleOp5361.closeClaw(sClawL, sClawR);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();
        telemetry.addData("Status", "Unsquishifying");
        compressor.setPosition(.98);//change if compressor malfunctions
        sleep(750);
        clawTower.setPower(-.2);
        sleep(400);
        clawTower.setPower(0);
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        //wait until the end
        while(runtime.seconds() < 27.5) {}
        telemetry.addData("Status", "Extending");
        telemetry.update();
        tapeExtender.setPower(.7);
        sleep(1500);
        tapeExtender.setPower(0);
    }
}
