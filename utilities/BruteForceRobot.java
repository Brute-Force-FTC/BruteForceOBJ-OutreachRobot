package org.firstinspires.ftc.bruteforce.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.exception.DuplicateNameException;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.exception.RobotProtocolException;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.io.PrintStream;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;



public class BruteForceRobot {
    public Orientation             lastAngles = new Orientation();
    private Blinker controlHub;
    private Blinker expansion_Hub_3;
    public BNO055IMU imu;
    public DcMotor frontLeft;
    public DcMotor frontRight; 
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor viperSlide;
    public CRServo claw;
    //private HardwareDevice webcam_1;
    
    
    

    static final double     COUNTS_PER_MOTOR_REV    = 1075.2 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_REV          = 10;
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     //98mm; For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     REVS_PER_SLIDE       = 3;
    static final double     COUNTS_PER_SLIDE     = (COUNTS_PER_REV * REVS_PER_SLIDE);
    public static final double DRIVE_SPEED = 0.40;
    public static final double TURN_SPEED = 0.40; 
    public static final double STRAFE_SPEED = 0.40;
    public static final double SLIDE_SPEED = 1;
    
    
    
    
    
    public BruteForceRobot(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor vs) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        viperSlide = vs;
    }
    
    public BruteForceRobot(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        //webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(CRServo.class, "claw");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        imu.initialize(parameters);
    }
    
    public double checkDirectionF(double gA, Telemetry telemetry)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.1;
        
        //sets the current angle to the new globalAngle after deltaAngle has been added
        angle = getAngle(gA, telemetry);
        
        //if there is no difference, then no correction neede
        //but if there is a difference, then it sets the correction to -angle
        //sets it to the inverse because the angle control is flipped from the motors
        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.
        
        //multiplies the correction by the gain to get smooth correction
        correction = correction * gain;

        //return the finished var, corrections
        return correction;
    }
    
    public double checkDirectionB(double gA, Telemetry telemetry)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.1;
        
        //sets the current angle to the new globalAngle after deltaAngle has been added
        angle = getAngle(gA, telemetry);
        
        //if there is no difference, then no correction neede
        //but if there is a difference, then it sets the correction to -angle
        //sets it to the inverse because the angle control is flipped from the motors
        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = angle;        // reverse sign of angle for correction.
        
        //multiplies the correction by the gain to get smooth correction
        correction = correction * gain;

        //return the finished var, corrections
        return correction;
    }
    
    
    public double getAngle(double gA, Telemetry telemetry)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        
        //returns absolute orientation of sensor as var, angles
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        //creates the var deltaAngle which takes the 
        //var angles, first movement - the last recorded angle before reset
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        telemetry.addData("delta:", deltaAngle);
        
        //if the delta angles diff. is above 180 degrees, then it adds 360
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        
        //ads the deltaAngle to the globalAngle
        gA += deltaAngle;
        
        //sets the lastAngle to the last orientation it got previously
        lastAngles = angles;
        
        //returns the finished globalAngle
        return gA;
    }
    
    public void resetAngle(double gA)
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gA = 0;
    }
    
    public void encoderSlide(int n) {
        //if (viperSlide.getCurrentPosition() > 4400 || viperSlide.getCurrentPosition() < 0) {
        //    throw new RuntimeException();
        //}
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setTargetPosition(-n);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(Math.abs(-1));
        while (viperSlide.isBusy()) {
        ;
        }
        viperSlide.setPower(0);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void holdViper(double p, double l) {
        if (viperSlide.getCurrentPosition() != l) {
            double error = l - viperSlide.getCurrentPosition();
            
            double out  = (p*error);
            
            viperSlide.setPower(out);
        }
    }

    public void holdViper(double p, double i, double d, double l) {

            //double derivative = 0;
            double integralSum = 0;

            double lastError = 0; 
            //double error = 0;
            //double out = 0;

            ElapsedTime timer = new ElapsedTime();

            //keeps the slider at the position and resistant to gravity
                    if (viperSlide.getCurrentPosition() != l) {

                        double error = l - viperSlide.getCurrentPosition();

                        double derivative = (error - lastError) / timer.seconds();

                        integralSum = integralSum + (error * timer.seconds());

                        double out = (p * error) + (i * integralSum) + (d * derivative);

                        viperSlide.setPower(out);

                        lastError = error;

                        timer.reset();
                    }
    }
    
    
    public void moveForward(double n) {
        moveVertical(-n);
    }
    
    public void moveBackward(double n) {
        moveVertical(n);
    }
    
    public void moveViper(double n) {
        moveSlider(n);
    }
    
    public void moveSlider(double n) {
        viperSlide.setPower(-n);
    }
    
    // If n is positive, move forward
    // If n is negative, move backward
    public void moveVertical(double n) {
        n = -1 * n;
        frontLeft.setPower(0.25 * n);
        frontRight.setPower(-0.25 * n);
        backLeft.setPower(0.25 * n);
        backRight.setPower(-0.25 * n);
    }
    
    public void moveLeft(double n) {
        moveHorizontal(-n);
    }
    
    public void moveRight(double n) {
        moveHorizontal(n); 
    }    
    
    // If n is positive, move left
    // If n is negative, move right
    public void moveHorizontal(double n) {
        n = -1 * n;
        frontLeft.setPower(-0.3 * n);
        frontRight.setPower(-0.3 * n);
        backLeft.setPower(0.3 * n);
        backRight.setPower(0.3 * n); 
    }    
    
    public void rotateRight (double n) {
        rotate(n);
    }
    
    public void rotateLeft (double n) {
        rotate(-n);
    }    
    
    public void rotate (double n) {
        n = -1 * n;
        frontLeft.setPower(-0.2 * n);
        frontRight.setPower(-0.2 * n);
        backLeft.setPower(-0.2 * n);
        backRight.setPower(-0.2 * n);
    }    
    
    
    public void stopMovement() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
    public void stopSlider(){
        viperSlide.setPower(0);
    }

    /*public void moveClaw(double o) {
        claw.setPower(o);
        //if (claw.getPower() > 0.7 || claw.getPower() < -0.5) {
        //    throw new RuntimeException();
        //}
    }*/

    public void moveClaw(double o) {
        claw.setPower(o);
    }
    
    
    public void robotStop() { 
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        viperSlide.setPower(0);
    }
    
 
    
    public void stopCoast() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Moves the claw
   
    
    public void reverseMotors() {
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void resetEncoders() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void targetPosition(int n) {
        frontLeft.setTargetPosition(n);
        frontRight.setTargetPosition(n);
        backLeft.setTargetPosition(n);
        backRight.setTargetPosition(n);
    }
    
    public void runPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void setPower(double n) {
        frontLeft.setPower(n);
        frontRight.setPower(n);
        backLeft.setPower(n);
        backRight.setPower(n);
    }
    
    public CRServo getClaw() {
        return claw;
    }
    
    public DcMotor getSlide() {
        return viperSlide;
    }
    
    
    public DcMotor getFrontLeft(){
        return frontLeft;
    }
    
       public DcMotor getFrontRight(){
        return frontRight;
    }

      public DcMotor getBackLeft(){
        return backLeft;
    }

      public DcMotor getBackRight(){
        return backRight;
    }
    
    public void encoderDriveIMU(
        String config,
        double Inches, double gA, double timeoutS, boolean opModeIsActive, Telemetry telemetry)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive) {
            
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;
            double speed = 0;
            double correction = 0;
            
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", gA);
            telemetry.addData("3 correction", correction);
            telemetry.update();
            
            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
                speed = DRIVE_SPEED;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
                speed = DRIVE_SPEED;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
                speed = TURN_SPEED;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
                speed = TURN_SPEED;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
                speed = STRAFE_SPEED;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
                speed = STRAFE_SPEED;
            }
            
            // Determine new target position, and pass to motor controller
            int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( 
                    frontLeft.isBusy() 
                    && frontRight.isBusy() 
                    && backLeft.isBusy() 
                    && backRight.isBusy()
                )  
            {
                if (config == "f") {
                    correction = checkDirectionF(gA, telemetry);
                    frontLeft.setPower(-(speed - correction));
                    frontRight.setPower((speed + correction));
                    backLeft.setPower(-(speed - correction));
                    backRight.setPower((speed + correction)); 
                } else if (config == "b") {
                    correction = checkDirectionB(gA, telemetry);
                    frontLeft.setPower((speed - correction));
                    frontRight.setPower(-(speed + correction));
                    backLeft.setPower((speed - correction));
                    backRight.setPower(-(speed + correction)); 
                } else if (config == "r") {
                    correction = checkDirectionF(gA, telemetry);
                    frontLeft.setPower(-(speed - correction));
                    frontRight.setPower(-(speed + correction));
                    backLeft.setPower((speed - correction));
                    backRight.setPower((speed + correction)); 
                } else if (config == "l") {
                    correction = checkDirectionB(gA, telemetry);
                    frontLeft.setPower((speed - correction));
                    frontRight.setPower((speed + correction));
                    backLeft.setPower(-(speed - correction));
                    backRight.setPower(-(speed + correction)); 
                }
                
                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            
        }
    
    }
    
    public void encoderDrive(
        String config,
        double Inches, double timeoutS, boolean opModeIsActive, Telemetry telemetry)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive) {
            
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;
            double speed = 0;
            
            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
                speed = DRIVE_SPEED;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
                speed = DRIVE_SPEED;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
                speed = TURN_SPEED;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
                speed = TURN_SPEED;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
                speed = STRAFE_SPEED;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
                speed = STRAFE_SPEED;
            }
            
            // Determine new target position, and pass to motor controller
            int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( 
                    frontLeft.isBusy() 
                    && frontRight.isBusy() 
                    && backLeft.isBusy() 
                    && backRight.isBusy()
                )  
            {
                // Display it for the driver.
                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            
        }
    
    }

    public void encoderDriveSpeed(
        String config,
        double Inches, double speed, double timeoutS, boolean opModeIsActive, Telemetry telemetry)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive) {
            
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;
            
            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            }
            
            // Determine new target position, and pass to motor controller
            int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( 
                    frontLeft.isBusy() 
                    && frontRight.isBusy() 
                    && backLeft.isBusy() 
                    && backRight.isBusy()
                )  
            {
                // Display it for the driver.
                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            
        }
    
    }
    
}
//zadders