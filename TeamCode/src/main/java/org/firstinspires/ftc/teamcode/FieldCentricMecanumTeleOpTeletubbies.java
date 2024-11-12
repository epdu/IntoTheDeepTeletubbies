package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricMecanumTeleOpTeletubbies")
public class FieldCentricMecanumTeleOpTeletubbies extends LinearOpMode {
    HardwareTeletubbies robot = new HardwareTeletubbies();
    public String fieldOrRobotCentric="robot";
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // tune the power for easy to control it
            double y = gamepad1.left_stick_y*(0.7); // Remember, Y stick value is reversed
            double x = - gamepad1.left_stick_x*(0.7);
            double rx = - gamepad1.right_stick_x*(0.5); //*(0.5) is fine

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.imu.resetYaw();
            }

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.LFMotor.setPower(frontLeftPower);
            robot.LBMotor.setPower(backLeftPower);
            robot.RFMotor.setPower(frontRightPower);
            robot.RBMotor.setPower(backRightPower);
        }
    }

}