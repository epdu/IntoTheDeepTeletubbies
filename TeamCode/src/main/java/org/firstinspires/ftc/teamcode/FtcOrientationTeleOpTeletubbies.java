package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "A FtcOrientationTeleOpTeletubbies")
public class FtcOrientationTeleOpTeletubbies extends LinearOpMode {
    public static final double DriveTrains_ReducePOWER =  0.5 ;
    HardwareTeletubbies robot = new HardwareTeletubbies();
    public String fieldOrRobotCentric="robot";
    boolean move = false;

    @Override public void runOpMode() {
        robot.init(hardwareMap);
//        robot.LFMotor = hardwareMap.dcMotor.get("LFMotor"); //control hub port 1
//        robot.LBMotor = hardwareMap.dcMotor.get("LBMotor"); //control hub port 0
//        robot.RFMotor = hardwareMap.dcMotor.get("RFMotor"); // expansion hub port 1
//        robot.RBMotor = hardwareMap.dcMotor.get("RBMotor"); // expansion hub port 0
//        robot.RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.liftMotorL = hardwareMap.get(DcMotor.class, "liftMotorL"); //control hub ort 2
//        robot.liftMotorR = hardwareMap.get(DcMotor.class, "liftMotorR");
//        int positionL = robot.liftMotorL.getCurrentPosition();
//        int positionR = robot.liftMotorR.getCurrentPosition();
//        robot.liftMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        robot.liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        robot.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.Claw = hardwareMap.servo.get("Claw"); // expansion hub servo port 2
//        robot.Claw.setPosition(0.72);
//        robot.ArmR = hardwareMap.servo.get("ArmR"); // expansion hub servo port 0
//        robot.ArmL = hardwareMap.servo.get("ArmL"); // expansion hub servo port 1
//        robot.ArmL.setDirection(Servo.Direction.REVERSE);
//        robot.ArmL.setPosition(0.927);
//        robot.Wrist = hardwareMap.servo.get("Wrist"); // expansion hub servo port 3
//        robot.Wrist.setPosition(1);
//        robot.Wrist.setDirection(Servo.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()) {
            liftArmHigh();
            moveDriveTrain();
            if (gamepad1.right_trigger > 0.3) { //open
           //     robot.Claw.setPosition(0.5);
            }if (gamepad1.left_trigger > 0.3) { //close
             //   robot.Claw.setPosition(0.72);


            }if (gamepad2.a  && !move) { //down
               // robot.Wrist.setPosition(1);
              //  robot.ArmL.setPosition(0.927);
            }
            if (gamepad2.y && !move) { // up
                //robot.ArmL.setPosition(0.1);
                //robot.Wrist.setPosition(0);

            }
            if (gamepad2.b && !move) { //up
                //robot.Wrist.setDirection(Servo.Direction.REVERSE);
                //robot.Wrist.setPosition(0);
            }
            if (gamepad2.x && !move) { //down
                //robot.Wrist.setPosition(1);
            }
            //for up




            //for down
        }
    }
    public void moveDriveTrain() {
        double y = gamepad1.left_stick_y;
        double x =- gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;
        robot.LFMotor.setPower(fl*DriveTrains_ReducePOWER);
        robot.LBMotor.setPower(bl*DriveTrains_ReducePOWER);
        robot.RFMotor.setPower(fr*DriveTrains_ReducePOWER);
        robot.RBMotor.setPower(br*DriveTrains_ReducePOWER);

    }
    public void liftArmHigh () {
        double liftArm_y = -gamepad2.left_stick_y;
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorL.setPower(liftArm_y*0.5);
        robot.liftMotorR.setPower(liftArm_y*0.5);
        robot.liftMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        //up joystick makes the slides rotate clockwise on the out right side
        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up


    }
}




