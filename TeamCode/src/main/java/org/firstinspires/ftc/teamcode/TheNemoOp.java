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

        //set up servos
        ServoEx scooperServo = new SimpleServo(hardwareMap, "handRotator", 0, 180);
        ServoEx armServo = new SimpleServo(hardwareMap, "armServo", 0 , 180);


        int scooperTargetPosition = 180;
        int armTargetPosition = 180;

        boolean scooperFlipped = false;
        boolean armFlipped = false;


//        airplaneMotor.setRunMode(Motor.RunMode.RawPower);

        //set up mecanum
        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        //set up controller
        GamepadEx controller1 = new GamepadEx(gamepad1);

        //set up buttons
        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader rightBumper = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader xButton = new ButtonReader(controller1, GamepadKeys.Button.X);
        ButtonReader yButton = new ButtonReader(controller1, GamepadKeys.Button.Y);

        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        TriggerReader leftTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.LEFT_TRIGGER);


        scooperServo.setPosition(scooperTargetPosition);

        //wait for the game to start
        waitForStart();
        runtime.reset();

        //run until the end of the match
        while (opModeIsActive()) {


            //read buttons
            leftBumper.readValue();
            rightBumper.readValue();
            xButton.readValue();
            yButton.readValue();


            //set movement speed
            double strafeSpeed = -controller1.getLeftX();
            double forwardSpeed = -controller1.getLeftY();
            double rotateSpeed =  -controller1.getRightX();


            // scooper mover
            if (xButton.wasJustPressed()) {
                if(scooperFlipped) {
                    scooperTargetPosition = 0;
                    scooperFlipped = false;
                } else {
                    scooperTargetPosition= 180;
                    scooperFlipped = true;
                }
            }

            //  arm mover
            if (yButton.wasJustPressed()) {
                if(armFlipped) {
                    armTargetPosition = 0;
                    armFlipped = false;
                } else {
                    armTargetPosition= 180;
                    armFlipped = true;
                }
            }

            if (rightTrigger.isDown()) {
                armTargetPosition += 5;
            } else if (leftTrigger.isDown()) {
                armTargetPosition -= 5;
            }


            // move robot
            driveBase.driveFieldCentric(strafeSpeed, forwardSpeed, rotateSpeed,0, false);

            scooperServo.setPosition(scooperTargetPosition);
            armServo.setPosition(armTargetPosition);

            // telemetry
//            telemetry.addData("Arm Position", armSwinger.getCurrentPosition());
//            telemetry.addData("Arm Target Position", motorTargetPosition);
//            telemetry.addData("Hand Swinger Position:", handRotator.getPosition());
//            telemetry.addData("Hand Swinger Target Position", handServoTargetPosition);
//            telemetry.addData("Hand Opener Position:", handOpener.getPosition());
//            telemetry.addData("Hand Opener Target Position:", handOpenerTargetPosition);
//            telemetry.addData("position", airplaneServo.getAngle());


            telemetry.update();


        }
    }
}
