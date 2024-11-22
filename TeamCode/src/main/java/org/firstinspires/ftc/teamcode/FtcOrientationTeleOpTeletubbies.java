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
    //    private static final int POSITION_Y = -600;
    private static final int POSITION_A_IN = 100; // horizontal  slides all the way in
    private static final int POSITION_Y_EXTRUDE = 800;//horizontal  slides all the way out
    private static final int POSITION_Y_EXTRUDE_MORE = 1000; //horizontal  slides all the way out
    private static final int POSITION_A_BOTTOM = 100; //horizontal  slides all the way out
    private static final int POSITION_Y_LOW = 800; // horizontal  slides all the way in
    private static final int POSITION_Y_HIGH = 1600;//horizontal  slides all the way out


    private static final double SLIDE_POWER_H = 0.8; // Adjust as needed
    private static final double SLIDE_POWER_V = 0.6; // Adjust as needed


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
         //   liftArmHigh();
            moveDriveTrain();
            // 3 prong claw
            if (gamepad1.left_trigger > 0.3 ) { //open
                robot.V4BL.setPosition(0.8); //  V4B put down
               robot.V4BR.setPosition(0.8); //V4B put down
 //               robot.Wrist.setPosition(0.8); // WRIST left 45 degree
//                robot.Claw.setPosition(0.6); // too big opening 3 prong claw -open good
                // robot.Claw.setPosition(0.6); // loony claw -open good
            }if (gamepad1.right_trigger > 0.4) { //close
                robot.V4BL.setPosition(0.2); // V4BL.setPosition(0.2) they are always the same value V4B rise up
                robot.V4BR.setPosition(0.2); //       V4BR.setPosition(0.8); they are always the same value // wrist goodV4B rise up
 //               robot.Wrist.setPosition(0.2); // WRIST right 45 degree
//                robot.Claw.setPosition(0.9); // 3 prong claw -close good
                // robot.Claw.setPosition(0.828); // loony claw -close 835  max good

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
            }//if (gamepad2.a && !move) { //all the way in
// //               moveHSlideToPosition(POSITION_A_IN);
//                moveVSlideToPosition(POSITION_A_BOTTOM);
//            }
//            if (gamepad2.y && !move) { // out controlled
// //               moveHSlideToPosition(POSITION_Y_EXTRUDE);
//                moveVSlideToPosition(POSITION_Y_HIGH);
//            }
//            if (gamepad2.dpad_left && !move) { //out controlled
//                moveHSlideToPosition(POSITION_Y_EXTRUDE_MORE);
//                moveVSlideToPosition(POSITION_Y_LOW);
//            }
 //      HAND SPECIALIST   48444442243  JULIA MAYBERRY


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
    private void moveHSlideToPosition ( int targetPosition){
//            robot.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("liftMotorR.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
        telemetry.update();
        robot.HSMotor.setTargetPosition(-targetPosition);
         robot.HSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.HSMotor.setPower(+SLIDE_POWER_H);

        move = true;

        while (robot.HSMotor.isBusy() &&  move) {
            // Wait until the motor reaches the target position
        }

        robot.HSMotor.setPower(0);
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        move = false;
    }

    private void moveVSlideToPosition ( int targetPosition){
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("liftMotorR.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
        telemetry.addData("liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
        telemetry.update();
        robot.VSMotorL.setTargetPosition(-targetPosition);
        robot.VSMotorR.setTargetPosition(-targetPosition);
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.VSMotorL.setPower(+SLIDE_POWER_V);
        robot.VSMotorR.setPower(+SLIDE_POWER_V);
        move = true;
        while (robot.VSMotorL.isBusy() && robot.VSMotorR.isBusy() && move) {
            // Wait until the motor reaches the target position
        }
//        while (robot.VSMotorR.isBusy() && move) {
 //           // Wait until the motor reaches the target position
 //       }

        robot.VSMotorL.setPower(0);
        robot.VSMotorR.setPower(0);
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        move = false;
    }

}




