package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TheNemoOp", group="Linear OpMode")
public class TheNemoOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //this stuff happens when INIT pressed
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();


        Motor frontLeftMotor = new Motor (hardwareMap, "fl");
        Motor frontRightMotor = new Motor (hardwareMap, "fr");
        Motor backLeftMotor = new Motor (hardwareMap, "bl");
        Motor backRightMotor = new Motor (hardwareMap, "br");
        Motor armSwinger = new Motor(hardwareMap, "armswinger");

        //SET THESE!!!!
        int targetPosition = 0;
        int MAX_ENCODER = 0;
        int MIN_ENCODER = -0;
        int fowardTarget = 0;
        int backwardsTarget = 0;

        //after we figure out positions add this line to before the game starts:
        // armSwinger.setTargetPosition(targetPosition);


        double speed;

        armSwinger.setRunMode(Motor.RunMode.PositionControl);
        armSwinger.setPositionCoefficient(.05);
        armSwinger.setPositionTolerance(3);
        armSwinger.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        GamepadEx controller1 = new GamepadEx(gamepad1);

        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader rightBumper = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader yButton = new ButtonReader(controller1, GamepadKeys.Button.Y);
        ButtonReader xButton = new ButtonReader(controller1, GamepadKeys.Button.X);
        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        armSwinger.resetEncoder();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //read buttons
            leftBumper.readValue();
            rightBumper.readValue();
            rightTrigger.readValue();

            //set movement speed
            double strafeSpeed = -controller1.getLeftX();
            double forwardSpeed = -controller1.getLeftY();
            double rotateSpeed =  -controller1.getRightX();
            double heading = -imu.getRotation2d().getDegrees();

           //set speed multiplier
            if(rightTrigger.isDown()) {
                speed = 0.25;
            } else {
                speed = 1.0;
            }

            //set arm target
            if(leftBumper.isDown()) {
                // Lower arm target
                targetPosition += 20 * speed;
            } else if(rightBumper.isDown()) {
                // Raise arm target
                targetPosition -= 20 * speed;
            }

            //set arm target shortcuts
            if(yButton.isDown()) {
                targetPosition = fowardTarget;
            }

            if (xButton.isDown()) {
                targetPosition = backwardsTarget;
            }

            //move arm target if out of bounds
            if(targetPosition > MAX_ENCODER) {
                targetPosition = MAX_ENCODER;
            }

            if(targetPosition < MIN_ENCODER) {
                targetPosition = MIN_ENCODER;
            }

           //break arm
            if (armSwinger.atTargetPosition()){
                armSwinger.stopMotor();
            } else {
                armSwinger.set(0.275);
            }


            //move robot
            driveBase.driveFieldCentric(strafeSpeed, forwardSpeed, rotateSpeed,heading, false);
            //move arm
            armSwinger.setTargetPosition(targetPosition);

            //telemetry
            telemetry.addData("Arm Position",armSwinger.getCurrentPosition());

            telemetry.update();

        }
    }
}
