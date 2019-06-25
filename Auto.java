package teamcode;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DcMotor;
import virtual_robot.hardware.DistanceSensor;
import virtual_robot.hardware.Servo;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class Auto extends function {

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        if(opModeIsActive()) {
            for (int i = pathx.length - 1; i > 0 && opModeIsActive(); i--) {
                gotoPoint(pathx, pathy, i, 1);
                if(i == 80){
                    gyroTurn(1,-45);
                    offSet = 90;
                }
            }
        }
    }
}
