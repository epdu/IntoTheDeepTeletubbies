package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "A IntoTheDeepTeleOpTeletubbies 11252024")
public class IntoTheDeepTeleOpTeletubbies extends LinearOpMode {
    public static final double DriveTrains_ReducePOWER =  0.75 ;
    HardwareTeletubbies robot = new HardwareTeletubbies();
    public String fieldOrRobotCentric="robot";
    boolean move = false;
   private static final int POSITION_A_IN = 100; // horizontal slides all the way in
    private static final int POSITION_Y_EXTRUDE = 800;//horizontal slides  out
    private static final int POSITION_X_EXTRUDE_MORE = 1000; //horizontal slides all the way out
    private static final int POSITION_A_BOTTOM = 100; //horizontal  slides all the way in
    private static final int POSITION_X_LOW = 800; // horizontal  slides up
    private static final int POSITION_Y_HIGH = 1600;//horizontal  slides all the way up
    private static final double SLIDE_POWER_H = 0.8; // Adjust as needed
    private static final double SLIDE_POWER_V = 0.6; // Adjust as needed

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            moveDriveTrain();
            // 3 prong claw
            if (gamepad1.left_trigger > 0.3 ) { //open
                robot.Claw.setPosition(0.6); // too big opening 3 prong claw -open good
//                 robot.Claw.setPosition(0.6); // loony claw -open good
            }if (gamepad1.right_trigger > 0.3) { //close
                robot.Claw.setPosition(0.9); // 3 prong claw -close good
//                 robot.Claw.setPosition(0.828); // loony claw -close 835  max good
            }//if (gamepad1.a  && !move) { //down
//                robot.V4BL.setPosition(0.49); //  V4B put down
//                robot.V4BR.setPosition(0.49); //V4B put down
//            }
//            if (gamepad2.y && !move) { // up
//                robot.V4BL.setPosition(0.32); // V4BL.setPosition(0.2) they are always the same value V4B rise up
//                robot.V4BR.setPosition(0.32); //       V4BR.setPosition(0.8); they are always the same value // wrist goodV4B rise up
//            }
//            if (gamepad1.b && !move) { //right
//                robot.Wrist.setPosition(0.35); // WRIST right 45 degree
//            }
//            if (gamepad1.x && !move) { //left
//                robot.Wrist.setPosition(0.65); // WRIST left 45 degree
//            }
//            if (gamepad1.right_bumper && !move) { //left
//                robot.Wrist.setPosition(0.5); // WRIST left 45 degree
//            }
//            if (gamepad2.a && !move) { //left
//                robot.V4BL.setPosition(0.4);
//                robot.V4BR.setPosition(0.4);
//            }




 //      HAND SPECIALIST   48444442243  JULIA MAYBERRY


            //for up




            //for down
        }
    }
    public void moveDriveTrain() {
        double y = gamepad1.left_stick_y;
        double x =- gamepad1.left_stick_x;
        double rx = (-gamepad1.right_stick_x*0.5);
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

    public void liftVertSlidesHigh () {
        double liftVertSlides_y = -gamepad2.left_stick_y;
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.VSMotorL.setPower(liftVertSlides_y*0.45);
        robot.VSMotorR.setPower(liftVertSlides_y*0.45);
        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        //up joystick makes the slides rotate clockwise on the out right side
        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up


    }

}




