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

    //this stuff happens when INIT pressed
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //set up revIMU
//        RevIMU imu = new RevIMU(hardwareMap);
//        imu.init();

        //set up motors
        Motor frontLeftMotor = new Motor (hardwareMap, "fl");
        Motor frontRightMotor = new Motor (hardwareMap, "fr");
        Motor backLeftMotor = new Motor (hardwareMap, "bl");
        Motor backRightMotor = new Motor (hardwareMap, "br");

        Motor armSwinger = new Motor(hardwareMap, "armswinger");
        Motor airplaneMotor = new Motor(hardwareMap, "airplaneMotor");


//        ServoEx airplaneServo = new SimpleServo(hardwareMap, "airplaneServo", 0, 180);

        int targetPosition = 0;  //where the arm starts
//        int MAX_ENCODER = 500;
//        int MIN_ENCODER = 20;
        int fowardTarget = 195;
        int backwardsTarget = 5;



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

        TriggerReader rightTrigger = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);

        armSwinger.setTargetPosition(targetPosition);

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

            //set movement speed
            double strafeSpeed = -controller1.getLeftX();
            double forwardSpeed = -controller1.getLeftY();
            double rotateSpeed =  -controller1.getRightX();
//            double heading = -imu.getRotation2d().getDegrees();

            if(bButton.isDown()){
                airplaneMotor.set(1.0);
            }else{
                airplaneMotor.set(0);
            }

//            if(aButton.isDown()){
//                airplaneServo.turnToAngle(180);
//            }


            //set arm target
            if(leftBumper.isDown()) {
                // Lower arm target
                targetPosition += 5;
            } else if(rightBumper.isDown()) {
                // Raise arm target
                targetPosition -= 5;
            }

            //set arm target shortcuts
            if(yButton.isDown()) {
                targetPosition = 510;
            }

            if (xButton.isDown()) {
                targetPosition = 30;
            }

//
//            if(targetPosition > MAX_ENCODER) {
//                targetPosition = MAX_ENCODER;
//            }
//
//            if(targetPosition < MIN_ENCODER) {
//                targetPosition = MIN_ENCODER;
//            }


            if (armSwinger.atTargetPosition()){
                armSwinger.stopMotor();
            } else {
                armSwinger.set(0.08);
            }



            //move robot
            driveBase.driveFieldCentric(strafeSpeed, forwardSpeed, rotateSpeed,0, false);
            //move arm
            armSwinger.setTargetPosition(targetPosition);

            //telemetry
            telemetry.addData("Arm Position", armSwinger.getCurrentPosition());
            telemetry.addData("Arm Target Position", targetPosition);

            telemetry.update();


        }
    }
}
