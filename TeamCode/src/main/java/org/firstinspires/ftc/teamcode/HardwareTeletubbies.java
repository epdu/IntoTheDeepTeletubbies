
       package org.firstinspires.ftc.teamcode;
       import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
       import com.qualcomm.robotcore.hardware.DcMotor;
       import com.qualcomm.robotcore.hardware.DcMotorSimple;
       import com.qualcomm.robotcore.hardware.HardwareMap;
       import com.qualcomm.robotcore.hardware.IMU;
       import com.qualcomm.robotcore.hardware.Servo;
       import com.qualcomm.robotcore.util.ElapsedTime;
       import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
       /**
        * This is NOT an opmode.
        *
        * This class can be used to define all the specific hardware for a single robot.
        * In this case that robot is powerpuffs`s robot from Pushbot.
        * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
        *
        * This hardware class assumes the following device names have been configured on the robot:
        * Note:  All names are lower case and some have single spaces between words.
        *    DcMotor RFMotor;
        *    DcMotor LFMotor;
        *    DcMotor RBMotor;
        *    DcMotor LBMotor;
        * RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        * LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        * RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        * LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        * Motor channel:  Left  drive motor:        "left_drive"
        * Motor channel:  Right drive motor:        "right_drive"
        * Motor channel:  Manipulator drive motor:  "left_arm"
        * Servo channel:  Servo to open left claw:  "left_hand"
        * Servo channel:  Servo to open right claw: "right_hand"
        *
        *
        */
//Good version 02082024
       public class HardwareTeletubbies
       {
           /* local OpMode members. */
           HardwareMap hwMap =  null;
           /* Public OpMode members. */
           public DcMotor RFMotor;
           public DcMotor LFMotor;
           public DcMotor RBMotor;
           public DcMotor LBMotor;
           public Servo Claw;
           public Servo Wrist;
           public Servo V4BL;
           public Servo V4BR;
           public DcMotor HSMotor; //horizontal Slides motor  extruder
           public DcMotor VSMotorL; //vertical Slides motor left
           public DcMotor VSMotorR; //vertical Slides motor right
           public DcMotor TMotor; // For testing
           public Servo ArmL;
           public Servo ArmR;
           public Servo TServo;

//           public Servo ArmL;
//           public DcMotor liftMotorL;
//           public DcMotor liftMotorR;

//           public Servo ClawR;
//           public Servo ClawL;

//           public Servo ArmR;
//           public Servo ArmL;
// //          public Servo Drone;
           IMU imu;
           public static final double DriveTrains_POWER       =  0.5 ;
           public static final double MID_SERVO       =  0.5 ;
           public static final double ARM_UP_POWER    =  0.45 ;
           public static final double ARM_DOWN_POWER  = -0.45 ;
           private ElapsedTime period  = new ElapsedTime();
           /* Constructor */
           /* Initialize standard Hardware interfaces */
           public void init(HardwareMap ahwMap) {
               // Save reference to Hardware map
               hwMap = ahwMap;
               // Define and Initialize Motors
               LFMotor   = hwMap.get(DcMotor.class, "LFMotor");//02022024 control hub port 0
               RFMotor  = hwMap.get(DcMotor.class, "RFMotor"); //02022024 control hub port 1
               LBMotor   = hwMap.get(DcMotor.class, "LBMotor");//02022024 control hub port 2
               RBMotor  = hwMap.get(DcMotor.class, "RBMotor");//02022024 control hub port 3
//               TMotor = hwMap.get(DcMotor.class, "TM");//02022024 control hub port 0 //only for motor program testing

               RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
               RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

               LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//               HSMotor = hwMap.get(DcMotor.class, "HSMotor");// expansion  hub? port 4
//               int positionH = HSMotor.getCurrentPosition();
//               HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
               VSMotorL = hwMap.get(DcMotor.class, "VSMotorL");// expansion  hub? port 0
               int positionVL = VSMotorL.getCurrentPosition();
               VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
               VSMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
               VSMotorR = hwMap.get(DcMotor.class, "VSMotorR");// expansion  hub? port 3
               int positionVR = VSMotorR.getCurrentPosition();
               VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

               // Set all motors to zero power
               setAllPower(0);

               // Set all motors to run without encoders.
               // May want to use RUN_USING_ENCODERS if encoders are installed.
               LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//               TServo= hwMap.get(Servo.class, "TS");//only for servo program testing
//               TServo.setPosition(0.5);//  good
//               TMotor = hwMap.get(DcMotor.class, "TM");//02022024 control hub port 0 //only for motor program testing
//               TMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//               TMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               // Set all motors to run without encoders.
               // May want to use RUN_USING_ENCODERS if encoders are installed.


//               Claw = hwMap.get(Servo.class, "Claw");//control hub port  good
//               Wrist = hwMap.get(Servo.class, "Wrist");//control hub port x
//               V4BR = hwMap.get(Servo.class, "V4BR");//control hub port
//               V4BL = hwMap.get(Servo.class, "V4BL");//control hub port
//               V4BL.setDirection(Servo.Direction.REVERSE);


//               V4BL.setPosition(0.32);//  good
//               V4BR.setPosition(0.32);//
 //             Wrist.setPosition(0.5);// wrist good
//                Claw.setPosition(0.9);// 3 prong claw good-


               //Claw.setPosition(0.828);// loony claw good-



//               Wrist = hwMap.get(Servo.class, "Wrist");//control hub port x
//               Wrist.setPosition(0.8);

////        ClawL.setDirection(Servo.Direction.REVERSE);
               //
//               Drone = hwMap.get(Servo.class, "Drone");//expan  hub port 5
//               Drone.setPosition(0);
//               ClawR = hwMap.get(Servo.class, "ClawR");//control hub port 2
//               ClawL = hwMap.get(Servo.class, "ClawL");//control hub port 3
//               ClawR.setPosition(0.71);
////        ClawL.setPosition(0.4);
//               ClawL.setPosition(0.08);
////        ClawL.setDirection(Servo.Direction.REVERSE);
//               Wrist = hwMap.get(Servo.class, "wrist");//control hub port 5
//               Wrist.setPosition(0.8);
////        Wrist.setPosition(0.34);
//               ArmL = hwMap.get(Servo.class, "ArmL");
//               ArmR = hwMap.get(Servo.class, "ArmR");
//               ArmL.setDirection(Servo.Direction.REVERSE);0000

               //inorder to reduce the ESD problems, we updated to be REV 9 axis imu with i2c port 1, imuinternal for the
               // REV control hub build in imu


               imu = hwMap.get(IMU.class, "imu");
               IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                       RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                       RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
               double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

               imu.initialize(parameters);
//        parameters.angleUnit           = IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.imuOrientationOnRobot = new JustLoggingAccelerationIntegrator();

               // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
               // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
               // and named "imu".

           }
           //Set power to all motors

           public void setMotorPower(double lF, double rF, double lB, double rB){
               LFMotor.setPower(lF*DriveTrains_POWER);
               LBMotor.setPower(lB*DriveTrains_POWER);
               RBMotor.setPower(rB*DriveTrains_POWER);
               RFMotor.setPower(rF*DriveTrains_POWER);
           }
           public void setAllPower(double p){
               setMotorPower(p,p,p,p);
           }
       }

