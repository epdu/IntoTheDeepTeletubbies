package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HardWare {

    HardwareMap hwMap = null;

    public DcMotor RFMotor;
    public DcMotor RBMotor;
    public DcMotor LFMotor;
    public DcMotor LBMotor;

    IMU imu;


    public void init(HardwareMap ahwMap){
        ahwMap = hwMap;

        RFMotor = hwMap.get(DcMotor.class, "RFMotor");
        RBMotor = hwMap.get(DcMotor.class, "RBMotor");
        LFMotor = hwMap.get(DcMotor.class, "LFMotor");
        LBMotor = hwMap.get(DcMotor.class, "LBMotor");

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setAllPower(0);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
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


    public void setMotorPower(double lF, double rF, double lB, double rB){
        LFMotor.setPower(lF);
        LBMotor.setPower(lB);
        RBMotor.setPower(rB);
        RFMotor.setPower(rF);
    }
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }


}
