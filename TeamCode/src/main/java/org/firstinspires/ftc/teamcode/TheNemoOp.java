

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;


@TeleOp(name="TheNemoOp", group="Linear OpMode")
public class TheNemoOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();


        Motor frontLeftMotor = new Motor (hardwareMap, "frontLeft");
        Motor frontRightMotor = new Motor (hardwareMap, "frontRight");
        Motor backLeftMotor = new Motor (hardwareMap, "backLeft");
        Motor backRightMotor = new Motor (hardwareMap, "backRight");

        MecanumDrive driveBase = new MecanumDrive (frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        GamepadEx controller1 = new GamepadEx(gamepad1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double strafeSpeed = -controller1.getLeftX();
            double forwardSpeed = controller1.getLeftY();
            double rotateSpeed =  -controller1.getRightX();
            double heading = -imu.getRotation2d().getDegrees();

            driveBase.driveFieldCentric(strafeSpeed, forwardSpeed, rotateSpeed,heading, false);


        }
    }
}
