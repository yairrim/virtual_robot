package teamcode;

import virtual_robot.hardware.*;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class Hardware {
    DcMotor backLeft   = null;
    DcMotor frontLeft  = null;
    DcMotor frontRight = null;
    DcMotor backRight  = null;
    BNO055IMU imu      = null;

    public void init(HardwareMap haMap){
        backLeft = haMap.dcMotor.get("back_left_motor");
        frontLeft = haMap.dcMotor.get("front_left_motor");
        frontRight = haMap.dcMotor.get("front_right_motor");
        backRight = haMap.dcMotor.get("back_right_motor");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //GyroSensor gyro = haMap.gyroSensor.get("gyro_sensor");
        imu = haMap.get(BNO055IMU.class, "imu");

        Servo backServo = haMap.servo.get("back_servo");
        DistanceSensor frontDistance = haMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = haMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = haMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = haMap.get(DistanceSensor.class, "back_distance");
        //gyro.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        ColorSensor colorSensor = haMap.colorSensor.get("color_sensor");
    }
}
