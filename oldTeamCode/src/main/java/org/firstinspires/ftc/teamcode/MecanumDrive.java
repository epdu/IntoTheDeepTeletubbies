package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumDrive extends LinearOpMode {
    DcMotor LFMotor;
    DcMotor LBMotor;
    DcMotor RFMotor;
    DcMotor RBMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        LFMotor = hardwareMap.dcMotor.get("LFMotor"); //control hub port 1
        LBMotor = hardwareMap.dcMotor.get("LBMotor"); //control hub port 0
        RFMotor = hardwareMap.dcMotor.get("RFMotor"); // expansion hub port 1
        RBMotor = hardwareMap.dcMotor.get("RBMotor"); // expansion hub port 0


        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo Claw = hardwareMap.servo.get("claw");

        waitForStart();


        while (opModeIsActive()) {
            moveDriveTrain();
            if (gamepad1.right_trigger > 0.3) { //close
                Claw.setPosition(0.4);
            }
            if (gamepad1.left_trigger > 0.3) { //open
                Claw.setPosition(0);
            }

        }
    }
    public void moveDriveTrain() {
        double y = gamepad1.left_stick_y;
        double x =- gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x*(0.5);





        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;


        LFMotor.setPower(fl*1);
        LBMotor.setPower(bl*1);
        RFMotor.setPower(fr*1);
        RBMotor.setPower(br*1);


    }
}
