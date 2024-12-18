package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.HardwareTeletubbies;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.FieldCentricMecanumTeleOpTeletubbies.DriveTrains_ReducePOWER;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@TeleOp(name = "A QualTeleOpTele 12152024 V0")
public class IntoTheDeepTeleOpTeletubbies extends LinearOpMode {
    public static final double DriveTrains_ReducePOWER = 0.75;
    HardwareTeletubbies robot = new HardwareTeletubbies();
    public String fieldOrRobotCentric = "robot";
    boolean move = false;
    private static final int POSITION_X_IN = 0; // horizontal slides all the way in
    private static final int POSITION_B_EXTRUDE = 400;//horizontal slides  out //600
    private static final int POSITION_B_EXTRUDE_MORE = 600; //horizontal slides all the way out 800
    private static final int POSITION_A_BOTTOM = 0; //Vertical  slides all the way in
    private static final int POSITION_Y_LOW = 800; // Vertical slides up //800 //1000 too high
    private static final int POSITION_Y_HIGH = 1250;//Vertical slides all the way up
    private static final int POSITION_Y_HIGHH = 1300;//Vertical slides all the way up
    private static final double SLIDE_POWER_H = 0.4; // Adjust as needed
    private static final double SLIDE_POWER_V = 0.70; // Adjust as needed
    private static final double SERVO_STEP = 0.01; // 每次调整的伺服步长
    double servoPosition = 0.5;
    private static final double SLIDE_POWER = 0.8; // Adjust as needed
    public float speedMultiplier = 0.5f;
    public float speedLimiter = 0.05f;
    int controlMode = 0;
    ButtonHandler dpadDownHandler = new ButtonHandler();
    ButtonHandler dpadUpHandler = new ButtonHandler();
    ButtonHandler leftBumperHandler = new ButtonHandler();
    ButtonHandler rightBumperHandler = new ButtonHandler();
    ButtonHandler gamepad1XHandler = new ButtonHandler();
    ButtonHandler gamepad1BHandler = new ButtonHandler();
    ButtonHandler gamepad1YHandler = new ButtonHandler();
    ButtonHandler gamepad1AHandler = new ButtonHandler();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
//            moveDriveTrain_RobotCentric(); // Select either RobotCentricDriveTrain() or FieldCentricDriveTrain() based on your requirements.
            moveDriveTrain_FieldCentric() ;
//            moveDriveTrain(); //robot centric
            liftVertSlidesHigh();
//            extrHoriSlidesLong();

//Begin Definition and Initialization of gamepad
            dpadDownHandler.update(gamepad1.dpad_down);
            dpadUpHandler.update(gamepad1.dpad_up);
            leftBumperHandler.update(gamepad1.left_bumper);
            rightBumperHandler.update(gamepad1.right_bumper);
            gamepad1XHandler.update(gamepad1.x);
            gamepad1BHandler.update(gamepad1.b);
            gamepad1YHandler.update(gamepad1.y);
            gamepad1AHandler.update(gamepad1.a);

        if (gamepad1.start) { // 切换控制模式
           controlMode = (controlMode + 1) % 2; // 假设两种模式 0 和 1
            telemetry.addData("Control Mode", controlMode == 0 ? "Mode 0: Standard" : "Mode 1: Advanced");
            telemetry.update();
            sleep(300); // 防止快速连击导致模式快速切换
        }
// 根据不同模式定义按键功能
            switch (controlMode) {
                case 0:
                    // intake
                    //Begin  moveHSlideToPosition
                    if (gamepad1BHandler.isShortPress()) { //IN
                        moveHSlideToPosition(POSITION_X_IN);
                    }
                    if (gamepad1XHandler.isShortPress()) { //EXTRUDE
                        moveHSlideToPosition(POSITION_B_EXTRUDE);
                    }
                    if (gamepad1XHandler.isDoubleClick()) { //EXTRUDE_MORE
                        moveHSlideToPosition(POSITION_B_EXTRUDE_MORE);
                    }
                    //End  moveHSlideToPosition

//Begin  open and close of intakeclaw 12122024 finetuned

                    if (gamepad1.left_trigger > 0.3 && gamepad1.left_trigger <= 0.7) { // 轻按
                        robot.IClaw.setPosition(0.32); //12122024
                    }
                    if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
                        robot.IClaw.setPosition(0.542); //0.54 moveable 0.542 barely movable 0.543 hold
                    }
                    if (gamepad1.right_trigger > 0.7) { // 深按
                        robot.IClaw.setPosition(0.543); //0.54 moveable 0.542 barely movable 0.543 hold
                    }

//End open and close of intakeclaw

//Begin  Wristzyaw
                    if (gamepad1.b) { //right
                        robot.Wristzyaw.setPosition(0.22); //Wristzyaw right 45 degree 12122024
                    }
                    if (gamepad1.x) { //left
                        robot.Wristzyaw.setPosition(0.5); // Wristzyaw left 45 degree 12122024 // robot.Wristzyaw.setPosition(0.65); for left
                    }

//one key ready for pick
                    if (gamepad1.right_bumper) { //up if arm is Horizontal, the the wrist is vertical up and down
                        robot.Wristxpitch.setPosition(0.65);
                        sleep(200);
                        robot.IClaw.setPosition(0.32);
                        sleep(200);
                        robot.IArmL.setPosition(0.7);
                        robot.IArmR.setPosition(0.7);
                    }

//one key ready for pick up

//one key ready for transfer
                    if (gamepad1.right_bumper) { //

                    robot.Wristxpitch.setPosition(0.35); // Wristxpitch
                        sleep(200);
                        robot.IArmL.setPosition(0.7);
                        robot.IArmR.setPosition(0.7);
                        sleep(200);
                        robot.OClaw.setPosition(0.32); //12122024
                        sleep(200);
                        robot.OArmL.setPosition(0.99);
                        robot.OArmR.setPosition(0.99);
                        sleep(200);
                        robot.OClaw.setPosition(0.548); // 0.543 hold
                        sleep(200);
                        robot.IClaw.setPosition(0.32); //12122024
                        robot.OArmL.setPosition(0.06);
                        robot.OArmR.setPosition(0.06);
                        sleep(200);
                        moveVSlideToPosition(-POSITION_Y_HIGH);// high

                    }

//one key ready for transfer

//Begin  IArm L and R

                    if (gamepad1.y) { //up
                        robot.IArmL.setPosition(0.6);  // always same as hardware IArmL.setPosition(0.6);
                        robot.IArmR.setPosition(0.6);
                    }
                    if (gamepad1.a ) { //down
                        robot.IArmL.setPosition(0.725);
                        robot.IArmR.setPosition(0.725); //
                    }

//end  IArm L and R


//Begin  open and close of outtakeclaw 12122024 finetuned

                    if (gamepad2.left_trigger > 0.3) { //open
                        robot.OClaw.setPosition(0.32); //12122024
                    }
                    if (gamepad2.right_trigger > 0.3) { //close
                        robot.OClaw.setPosition(0.548); // 0.543 hold
                    }

//End open and close of outtakeclaw

//End Definition and Initialization of gamepad


//Begin debugging with a step increment of 0.05

/**
 * This code snippet controls the position of a servo motor using the gamepad triggers.
 *
 * **Purpose**:
 * - The left trigger (`gamepad1.left_trigger`) increases the servo's position by a fixed step (`SERVO_STEP`).
 * - The right trigger (`gamepad1.right_trigger`) decreases the servo's position by a fixed step (`SERVO_STEP`).
 * - The servo position is constrained between 0.01 (minimum) and 0.99 (maximum) to prevent invalid values.
 * - The current servo position is displayed on the telemetry for real-time monitoring.
 *
 * **Usage Instructions**:
 * 1. Press the **left trigger** (`gamepad1.left_trigger`) to move the servo incrementally towards its maximum position.
 * 2. Press the **right trigger** (`gamepad1.right_trigger`) to move the servo incrementally towards its minimum position.
 * 3. The servo's position is updated with a small delay (`sleep(200)` milliseconds) to prevent rapid changes from multiple trigger presses.
 * 4. Adjust `SERVO_STEP` as needed to control the increment size for finer or coarser adjustments.
 *
 * **Setup**:
 * - Ensure the servo is connected to the correct port and initialized in the `robot.TServo` variable.
 * - Configure the `SERVO_STEP` variable to determine how much the position changes with each trigger press.
 * - Calibrate the servo movement range (e.g., 0.01 to 0.99) based on your servo's physical limits to avoid damage.
 */


//            if (gamepad1.left_trigger > 0.3) {
//                servoPosition = servoPosition + SERVO_STEP;
//                if (servoPosition >= 1.0) {
//                    servoPosition = 0.99; // 限制最大值
//                }
//                robot.TServo.setPosition(servoPosition);
//                telemetry.addData("Servo Position", servoPosition);
//                telemetry.update();
//                sleep(200);
//            }
//            if (gamepad1.right_trigger > 0.3) {
//                servoPosition = servoPosition - SERVO_STEP;
//                if (servoPosition <= 0.0) {
//                    servoPosition = 0.01; // 限制最小值
//                }
//                robot.TServo.setPosition(servoPosition);
//                telemetry.addData("Servo Position", servoPosition);
//                telemetry.update();
//                sleep(200);
//            }

//End debugging with a step increment of 0.05

                    //Begin  Wristxpitch do not use it any more
//            if (gamepad1.dpad_up && !move) { //up if arm is Horizontal, the the wrist is vertical up and down
//                robot.Wristxpitch.setPosition(0.05); // Wristxpitch  12122024
//            }
//            if (gamepad1.dpad_down && !move) { //down
//                robot.Wristxpitch.setPosition(0.65); // Wristxpitch 12122024 0.65
//            }
//End   Wristxpitch
//            if (gamepad2.a && !move ) { //down
//                moveVSlideToPosition(POSITION_A_BOTTOM);
//            }
//            if (gamepad2.y && !move) { //up prepare forchameber
//                moveVSlideToPosition(-POSITION_Y_LOW);
//            }
//            if (gamepad2.right_bumper && !move){ //upforchameber
//                moveVSlideToPosition(-POSITION_Y_HIGH);
//            }

                    //      HAND SPECIALIST   48444442243  JULIA MAYBERRY
                    //for up
                    //for down

                    // ...（其他原始代码逻辑保持不变）
                    break;

                case 1:
                    // out take
                    //Begin  moveVSlideToPosition

                    // 左触发器双功能：轻按和深按
                    if (gamepad1BHandler.isShortPress()) { //IN
                        moveVSlideToPosition(POSITION_A_BOTTOM);// slides down
                    }
                    if (gamepad1XHandler.isShortPress()) { //EXTRUDE
                        moveVSlideToPosition(-POSITION_Y_LOW);// slides move to middle
                    }
                    if (gamepad1XHandler.isDoubleClick()) { //EXTRUDE_MORE
                        moveVSlideToPosition(-POSITION_Y_HIGH);// high
                    }
                    if (gamepad1XHandler.isLongPress()) { //EXTRUDE_MORE
                        moveVSlideToPosition(-POSITION_Y_HIGHH);// very high
                    }

                    //End  moveVSlideToPosition

//one key ready for pick
                    if (gamepad1.right_bumper) { //up if arm is Horizontal, the the wrist is vertical up and down
                        robot.OArmL.setPosition(0.06);
                        robot.OArmR.setPosition(0.06);
                        sleep(200);
                        robot.IClaw.setPosition(0.32);
                        sleep(200);
                    }

//one key ready for pick up



//Begin  OArm L and R

                    if (gamepad1.y) { //rear specimen
                        robot.OArmL.setPosition(0.06);
                        robot.OArmR.setPosition(0.06);
                    }
                    if (gamepad1.a) { //front transfer
                        robot.OArmL.setPosition(0.99);
                        robot.OArmR.setPosition(0.99);
                    }

//end  OArm L and R

//Begin  open and close of outtakeclaw 12122024 finetuned

                    if (gamepad1.left_trigger > 0.3 && gamepad1.left_trigger <= 0.7) { // 轻按
                        robot.IClaw.setPosition(0.32); //12122024
                    }
                    if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
                        robot.IClaw.setPosition(0.548);
                    }
                    if (gamepad1.right_trigger > 0.7) { // 深按
                        robot.IClaw.setPosition(0.549); //
                    }



//End open and close of outtakeclaw

//End Definition and Initialization of gamepad


//Begin debugging with a step increment of 0.05

/**
 * This code snippet controls the position of a servo motor using the gamepad triggers.
 *
 * **Purpose**:
 * - The left trigger (`gamepad1.left_trigger`) increases the servo's position by a fixed step (`SERVO_STEP`).
 * - The right trigger (`gamepad1.right_trigger`) decreases the servo's position by a fixed step (`SERVO_STEP`).
 * - The servo position is constrained between 0.01 (minimum) and 0.99 (maximum) to prevent invalid values.
 * - The current servo position is displayed on the telemetry for real-time monitoring.
 *
 * **Usage Instructions**:
 * 1. Press the **left trigger** (`gamepad1.left_trigger`) to move the servo incrementally towards its maximum position.
 * 2. Press the **right trigger** (`gamepad1.right_trigger`) to move the servo incrementally towards its minimum position.
 * 3. The servo's position is updated with a small delay (`sleep(200)` milliseconds) to prevent rapid changes from multiple trigger presses.
 * 4. Adjust `SERVO_STEP` as needed to control the increment size for finer or coarser adjustments.
 *
 * **Setup**:
 * - Ensure the servo is connected to the correct port and initialized in the `robot.TServo` variable.
 * - Configure the `SERVO_STEP` variable to determine how much the position changes with each trigger press.
 * - Calibrate the servo movement range (e.g., 0.01 to 0.99) based on your servo's physical limits to avoid damage.
 */


//            if (gamepad1.left_trigger > 0.3) {
//                servoPosition = servoPosition + SERVO_STEP;
//                if (servoPosition >= 1.0) {
//                    servoPosition = 0.99; // 限制最大值
//                }
//                robot.TServo.setPosition(servoPosition);
//                telemetry.addData("Servo Position", servoPosition);
//                telemetry.update();
//                sleep(200);
//            }
//            if (gamepad1.right_trigger > 0.3) {
//                servoPosition = servoPosition - SERVO_STEP;
//                if (servoPosition <= 0.0) {
//                    servoPosition = 0.01; // 限制最小值
//                }
//                robot.TServo.setPosition(servoPosition);
//                telemetry.addData("Servo Position", servoPosition);
//                telemetry.update();
//                sleep(200);
//            }

//End debugging with a step increment of 0.05

                    //Begin  Wristxpitch do not use it any more
//            if (gamepad1.dpad_up && !move) { //up if arm is Horizontal, the the wrist is vertical up and down
//                robot.Wristxpitch.setPosition(0.05); // Wristxpitch  12122024
//            }
//            if (gamepad1.dpad_down && !move) { //down
//                robot.Wristxpitch.setPosition(0.65); // Wristxpitch 12122024 0.65
//            }
//End   Wristxpitch
//            if (gamepad2.a && !move ) { //down
//                moveVSlideToPosition(POSITION_A_BOTTOM);
//            }
//            if (gamepad2.y && !move) { //up prepare forchameber
//                moveVSlideToPosition(-POSITION_Y_LOW);
//            }
//            if (gamepad2.right_bumper && !move){ //upforchameber
//                moveVSlideToPosition(-POSITION_Y_HIGH);
//            }

                    //      HAND SPECIALIST   48444442243  JULIA MAYBERRY
                    //for up
                    //for down


                    break;

                // 如果需要更多模式，可以继续添加 case。
            }


////////////////////////////////////







        } //end of while loop
    } //end of run mode

    public void moveDriveTrain() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = (gamepad1.right_stick_x*0.5);
        double fl = y - x - rx;
        double bl = y + x - rx;
        double fr = y + x + rx;
        double br = y - x + rx;
        robot.LFMotor.setPower(fl*DriveTrains_ReducePOWER);
        robot.LBMotor.setPower(bl*DriveTrains_ReducePOWER);
        robot.RFMotor.setPower(fr*DriveTrains_ReducePOWER);
        robot.RBMotor.setPower(br*DriveTrains_ReducePOWER);
    }

//Begin Definition and Initialization of Vertical Slides by gamepad2.left_stick_y

    public void liftVertSlidesHigh () {
        double liftVertSlides_y = -gamepad2.left_stick_y;
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.VSMotorL.setPower(liftVertSlides_y*0.45);
        robot.VSMotorR.setPower(liftVertSlides_y*0.45);
        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        //up joystick makes the slides rotate clockwise on the out right side
        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up
    }

//End Definition and Initialization of Vertical Slides by gamepad2.left_stick_y



//Begin Definition and Initialization of Horizontal Slides by gamepad2.left_stick_x  extrude slides long

    public void extrHoriSlidesLong() {
        double liftVertSlides_y = gamepad2.left_stick_x;
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.HSMotor.setPower(liftVertSlides_y*0.45);
        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        //up joystick makes the slides rotate clockwise on the out right side
        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up
    }

//End Definition and Initialization of Horizontal Slides by gamepad2.left_stick_x

//Begin Definition and Initialization of Vertical Slides
    private void moveVSlideToPosition ( int targetPosition){
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("liftMotorL.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
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
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("after while liftMotorL.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
        telemetry.addData("after while liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
        telemetry.update();

        robot.VSMotorL.setPower(0);
        robot.VSMotorR.setPower(0);
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        move = false;
    }

//End Definition and Initialization of Vertical Slides

//Begin Definition and Initialization of Horizontal Slides
    private void moveHSlideToPosition ( int targetPosition){
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("robot.HSMotor.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
        telemetry.update();
        robot.HSMotor.setTargetPosition(-targetPosition);
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.HSMotor.setPower(+SLIDE_POWER_H);
        move = true;
        while (robot.HSMotor.isBusy()  && move) {
            // Wait until the motor reaches the target position
        }
//        while (robot.VSMotorR.isBusy() && move) {
        //           // Wait until the motor reaches the target position
        //       }
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("after while HSMotor.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
        telemetry.update();
        robot.HSMotor.setPower(0);
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        move = false;
    }

//End Definition and Initialization of Horizontal Slides



    public void moveDriveTrain_FieldCentric() {
        double y = gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * (1);
        double rx = -gamepad1.right_stick_x * (1); //*(0.5) is fine

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.back) {
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

        robot.LFMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
        robot.LBMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
        robot.RFMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
        robot.RBMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
    }

    public void moveDriveTrain_RobotCentric() {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x*0.5; // If a smooth turn is required 0.5

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.LFMotor.setPower(fl * speedMultiplier);
        robot.LBMotor.setPower(bl * speedMultiplier);
        robot.RFMotor.setPower(fr * speedMultiplier);
        robot.RBMotor.setPower(br * speedMultiplier);

    }
    public void RobotCentricDriveTrain () {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x*0.5; // If a smooth turn is required 0.5

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.LFMotor.setPower(fl * speedMultiplier);
        robot.LBMotor.setPower(bl * speedMultiplier);
        robot.RFMotor.setPower(fr * speedMultiplier);
        robot.RBMotor.setPower(br * speedMultiplier);

    }

//Begin Definition and Initialization of Horizontal Slides


//    private void moveHSlideToPosition ( int targetPosition){
//
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("targetPosition", targetPosition);
//        telemetry.addData("liftMotorR.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
//        telemetry.update();
//        robot.HSMotor.setTargetPosition(-targetPosition);
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.HSMotor.setPower(+SLIDE_POWER_H);
//
//        move = true;
//
//        while (robot.HSMotor.isBusy() &&  move) {
//            // Wait until the motor reaches the target position
//        }
//
//        robot.HSMotor.setPower(0);
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        move = false;
//    }


//End Definition and Initialization of Horizontal Slides




}








