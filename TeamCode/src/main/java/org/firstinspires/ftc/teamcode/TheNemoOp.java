package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TheNemoOp", group="Linear OpMode")
public class TheNemoOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //this stuff happens when init pressed
    @Override
    public void runOpMode() {

        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //set up motors
        Motor frontLeftMotor = new Motor (hardwareMap, "fl");
        Motor frontRightMotor = new Motor (hardwareMap, "fr");
        Motor backLeftMotor = new Motor (hardwareMap, "bl");
        Motor backRightMotor = new Motor (hardwareMap, "br");
        Motor armSwinger = new Motor(hardwareMap, "armswinger");
        Motor airplaneMotor = new Motor(hardwareMap, "airplaneMotor");

        //set up servos
        ServoEx handRotator = new SimpleServo(hardwareMap, "handRotator", 0, 180);
        ServoEx airplaneServo = new SimpleServo(hardwareMap, "airplaneServo", 0, 180);
        ServoEx handOpener = new SimpleServo(hardwareMap, "handOpener",0, 180);
        airplaneServo.setInverted(true);

        int motorTargetPosition = 0;
        int handServoTargetPosition = 180;
        int handOpenerTargetPosition = 0;
        int forwardTarget = 510;
        int backwardsTarget = 30;
        boolean handOpened = false;
        boolean handFlipped = false;


        armSwinger.setRunMode(Motor.RunMode.PositionControl);
        armSwinger.setPositionCoefficient(.05);
        armSwinger.setPositionTolerance(3);
        armSwinger.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        airplaneMotor.setRunMode(Motor.RunMode.RawPower);

        //set up mecanum
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        //set up controller
        GamepadEx controller1 = new GamepadEx(gamepad1);

        //set up buttons
        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader rightBumper = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader yButton = new ButtonReader(controller1, GamepadKeys.Button.Y);
        ButtonReader xButton = new ButtonReader(controller1, GamepadKeys.Button.X);
        ButtonReader bButton = new ButtonReader(controller1, GamepadKeys.Button.B);
        ButtonReader aButton = new ButtonReader(controller1, GamepadKeys.Button.A);
        ButtonReader leftDPad = new ButtonReader(controller1, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader rightDPad = new ButtonReader(controller1, GamepadKeys.Button.DPAD_RIGHT);
        ButtonReader downDPad = new ButtonReader(controller1, GamepadKeys.Button.DPAD_DOWN);
        ButtonReader upDPad = new ButtonReader(controller1, GamepadKeys.Button.DPAD_UP);

        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);

        armSwinger.setTargetPosition(motorTargetPosition);
        handRotator.setPosition(handServoTargetPosition);

        //wait for the game to start
        waitForStart();
        runtime.reset();

        armSwinger.resetEncoder();

        //run until the end of the match
        while (opModeIsActive()) {

            telemetry.update();

            //read buttons
            leftBumper.readValue();
            rightBumper.readValue();
            rightTrigger.readValue();
            leftDPad.readValue();
            rightDPad.readValue();

            //set movement speed
            double strafeSpeed = -controller1.getLeftX();
            double forwardSpeed = -controller1.getLeftY();
            double rotateSpeed =  -controller1.getRightX();


            //airplane servo set
            if(rightTrigger.isDown()){
                airplaneServo.turnToAngle(180);
                airplaneMotor.set(1.0);
            }




            //arm mover
            if(leftBumper.isDown()) {
                // Lower arm target
                motorTargetPosition += 5;
            } else if(rightBumper.isDown()) {
                // Raise arm target
                motorTargetPosition -= 5;
            }

//            //arm target shortcuts
//            if(aButton.isDown()) {
//                if (motorTargetPosition == forwardTarget){
//                    motorTargetPosition = backwardsTarget;
//                } else {
//                    motorTargetPosition = forwardTarget;
//                }
//
//            }

            // hand mover
            if (xButton.isDown()) {
                if(handFlipped) {
                    handServoTargetPosition = 0;
                    handFlipped = false;
                } else {
                    handServoTargetPosition= 180;
                    handFlipped = true;

                }

            }

            if (bButton.isDown()) {
                if(handOpened){
                    handOpenerTargetPosition = 0;
                    handOpened = false;
                } else {
                    handOpenerTargetPosition = 180;
                    handOpened = true;
                }
            }



            //motor safety
            if(motorTargetPosition > forwardTarget) {
                motorTargetPosition = forwardTarget;
            }

            if(motorTargetPosition < backwardsTarget) {
                motorTargetPosition = backwardsTarget;
            }


            if (armSwinger.atTargetPosition()){
                armSwinger.stopMotor();
            } else {
                armSwinger.set(0.08);
            }

            //move robot
            driveBase.driveFieldCentric(strafeSpeed, forwardSpeed, rotateSpeed,0, false);

            //move arm and hand servo
            armSwinger.setTargetPosition(motorTargetPosition);
            handRotator.setPosition(handServoTargetPosition);
            handOpener.setPosition(handOpenerTargetPosition);

            //telemetry
            telemetry.addData("Arm Position", armSwinger.getCurrentPosition());
            telemetry.addData("Arm Target Position", motorTargetPosition);
            telemetry.addData("Hand Swinger Position:", handRotator.getPosition());
            telemetry.addData("Hand Swinger Target Position", handServoTargetPosition);
            telemetry.addData("Hand Opener Position:", handOpener.getPosition());
            telemetry.addData("Hand Opener Target Position:", handOpenerTargetPosition);



            telemetry.update();


        }
    }
}
