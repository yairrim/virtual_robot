package teamcode;

import com.sun.javafx.geom.Vec2d;
import virtual_robot.controller.LinearOpMode;
import virtual_robot.hardware.DcMotor;
import virtual_robot.util.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public abstract class function extends LinearOpMode {
    Hardware robot = new Hardware();

    static final double     DISTANCE_TO_DOT         = 1.05;// origin :  0.72;
    static double     offSet         = 0;// origin :  0.72;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final int[]      pathx                   = {21,20,19,18,17,16,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,15,15,15,16,16,17,18,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,40,40,41,41,42,43,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80};
    static final int[]      pathy                   = {181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80};
//    static final int[]      pathx = {1,2,3,4,5,6,7,8,9};
//    static final int[]      pathy = {1,2,3,4,5,6,7,8,9};
    enum Direction{
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }
    public void runOpMode(){
    }

    public void gotoPoint(int[] pathX,int[] pathY,int currentIndex,double speed){
        Vec2d current = new Vec2d(pathX[currentIndex],pathY[currentIndex]);
        Vec2d next = new Vec2d(pathX[currentIndex - 1],pathY[currentIndex - 1]);
        int adjacent = (int)(next.x - current.x);
        int opposite = (int)(next.y - current.y);
        double angle = -offSet/2;
        gyroTurn(1,angle);
        gyroDrive(1,DISTANCE_TO_DOT,angle,adjacent,opposite);
    }












    public void gyroDrive ( double speed,
                            double distance,
                            double angle,double X,double Y) {
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        int Degree;
        double DrivePower;
        double x = X;
        double y = Y;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newBackLeftTarget = Math.abs(robot.backLeft.getCurrentPosition()) + moveCounts;
            newBackRightTarget = Math.abs(robot.backRight.getCurrentPosition()) + moveCounts;

            newFrontLeftTarget = Math.abs(robot.frontLeft.getCurrentPosition()) + moveCounts;
            newFrontRightTarget = Math.abs(robot.frontRight.getCurrentPosition()) + moveCounts;

            newBackLeftTarget  = Math.abs(newBackLeftTarget);
            newBackRightTarget = Math.abs(newBackRightTarget);
            newFrontLeftTarget  = Math.abs(newFrontLeftTarget);
            newFrontRightTarget = Math.abs(newFrontRightTarget);


            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (       Math.abs(robot.backLeft.getCurrentPosition()) < newBackLeftTarget &&
                            Math.abs(robot.backRight.getCurrentPosition()) < newBackRightTarget &&
                                    Math.abs(robot.frontLeft.getCurrentPosition()) < newFrontLeftTarget &&
                                            Math.abs(robot.frontRight.getCurrentPosition()) < newFrontRightTarget)) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                y = -Y;
                x = -X;

                telemetry.addData("X>",x);
                telemetry.addData("Y>",y);
                Degree = (int) (Math.toDegrees(Math.atan2(y, x)) + 90 + getHeading());

                telemetry.addData("Angle",Degree);
                Degree+=90;

                DrivePower = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));


                y = -(DrivePower * Math.sin(Math.toRadians(Degree)));
                x = -(DrivePower * Math.cos(Math.toRadians(Degree)));


                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = x;// - steer;
                rightSpeed = y;// + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                drive(leftSpeed,rightSpeed);
                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            drive(0,0);

        }
    }

    public void drive(double power1, double power2){
        robot.backLeft.setPower(power1);
        robot.backRight.setPower(power2);
        robot.frontLeft.setPower(power2);
        robot.frontRight.setPower(power1);
    }

    public void turn(double powerLeft, double powerRight){
        robot.backLeft.setPower(powerLeft);
        robot.backRight.setPower(powerRight);
        robot.frontLeft.setPower(powerLeft);
        robot.frontRight.setPower(powerRight);
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return error * PCoeff;
    }
    public double getHeading(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return orientation.firstAngle * 180.0 / Math.PI;
    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        drive(0,0);
    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        turn(leftSpeed,rightSpeed);
        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
}
