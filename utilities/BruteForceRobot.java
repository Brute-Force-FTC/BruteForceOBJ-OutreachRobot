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
    private Blinker expansionHub;
    public BNO055IMU imu;
    public DcMotor frontLeft;
    public DcMotor frontRight; 
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor viperSlide;
    public DcMotor viperSlide2;
    public CRServo claw;
    public DcMotor clawRot;
    private HardwareDevice webcam_1;
    
    

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
        expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(CRServo.class, "clawServo");
        clawRot = hardwareMap.get(DcMotor.class, "clawRot");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        viperSlide2 = hardwareMap.get(DcMotor.class, "viperSlide2");
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
    

    public void encoderSlideBoth(int n, int i) {
        //if (viperSlide.getCurrentPosition() > 4400 || viperSlide.getCurrentPosition() < 0) {
        //    throw new RuntimeException();
        //}
        RobotLog.d("Slider Before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider After RUN_USING_ENCODER and Before setTargetPosition; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setTargetPosition(-n);
        viperSlide2.setTargetPosition(i);
        RobotLog.d("Slider After setTargetPosition and Before RUN_TO_POSITION; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Slider After RUN_TO_POSITION and Before Setting Power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(Math.abs(-0.75));
        viperSlide2.setPower(Math.abs(0.75));
        RobotLog.d("Slider after setting power and before isBusy; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        while (viperSlide.isBusy() && viperSlide2.isBusy()) {
        ;
        }
        RobotLog.d("Slider after isBusy and before 0 power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(0);
        viperSlide2.setPower(0);
        RobotLog.d("Slider after 0 power and before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider after RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void encoderSlideBothSpeed(int n, int i, double s) {
        //if (viperSlide.getCurrentPosition() > 4400 || viperSlide.getCurrentPosition() < 0) {
        //    throw new RuntimeException();
        //}
        RobotLog.d("Slider Before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider After RUN_USING_ENCODER and Before setTargetPosition; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setTargetPosition(-n);
        viperSlide2.setTargetPosition(i);
        RobotLog.d("Slider After setTargetPosition and Before RUN_TO_POSITION; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Slider After RUN_TO_POSITION and Before Setting Power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(Math.abs(-s));
        viperSlide2.setPower(Math.abs(s));
        RobotLog.d("Slider after setting power and before isBusy; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        while (viperSlide.isBusy() && viperSlide2.isBusy()) {
        ;
        }
        RobotLog.d("Slider after isBusy and before 0 power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(0);
        viperSlide2.setPower(0);
        RobotLog.d("Slider after 0 power and before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider after RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void encoderSlide(int n) {
        //if (viperSlide.getCurrentPosition() > 4400 || viperSlide.getCurrentPosition() < 0) {
        //    throw new RuntimeException();
        //}
        RobotLog.d("Slider Before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider After RUN_USING_ENCODER and Before setTargetPosition; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setTargetPosition(-n);
        RobotLog.d("Slider After setTargetPosition and Before RUN_TO_POSITION; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Slider After RUN_TO_POSITION and Before Setting Power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(Math.abs(-1));
        RobotLog.d("Slider after setting power and before isBusy; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        while (viperSlide.isBusy()) {
        ;
        }
        RobotLog.d("Slider after isBusy and before 0 power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(0);
        RobotLog.d("Slider after 0 power and before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider after RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void encoderSlide2(int n) {
        //if (viperSlide.getCurrentPosition() > 4400 || viperSlide.getCurrentPosition() < 0) {
        //    throw new RuntimeException();
        //}
        RobotLog.d("Slider Before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider After RUN_USING_ENCODER and Before setTargetPosition; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide2.setTargetPosition(n);
        RobotLog.d("Slider After setTargetPosition and Before RUN_TO_POSITION; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Slider After RUN_TO_POSITION and Before Setting Power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide2.setPower(Math.abs(1));
        RobotLog.d("Slider after setting power and before isBusy; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        while (viperSlide2.isBusy()) {
        ;
        }
        RobotLog.d("Slider after isBusy and before 0 power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide2.setPower(0);
        RobotLog.d("Slider after 0 power and before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider after RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void encoderRot(int n) {
        //if (viperSlide.getCurrentPosition() > 4400 || viperSlide.getCurrentPosition() < 0) {
        //    throw new RuntimeException();
        //}
        RobotLog.d("Slider Before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        clawRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider After RUN_USING_ENCODER and Before setTargetPosition; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        clawRot.setTargetPosition(n);
        RobotLog.d("Slider After setTargetPosition and Before RUN_TO_POSITION; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        clawRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Slider After RUN_TO_POSITION and Before Setting Power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        clawRot.setPower(Math.abs(0.75));
        RobotLog.d("Slider after setting power and before isBusy; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        while (clawRot.isBusy()) {
        ;
        }
        RobotLog.d("Slider after isBusy and before 0 power; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        clawRot.setPower(0);
        RobotLog.d("Slider after 0 power and before RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        clawRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Slider after RUN_USING_ENCODER; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void holdVipers(double p, double l, double l2) {
        if (viperSlide.getCurrentPosition() != l) {
            double error = l - viperSlide.getCurrentPosition();
            
            double out  = (p*error);
            
            viperSlide.setPower(out);
        }
        
        if (viperSlide2.getCurrentPosition() != l2) {
            double error2 = l - viperSlide2.getCurrentPosition();
            
            double out2 = (p*error2);
            
            viperSlide2.setPower(out2);
        }
    }

    public void holdViper(double p, double i, double d, double l, double l2) {

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

            //double derivative = 0;
            double integralSum2 = 0;

            double lastError2 = 0; 
            //double error = 0;
            //double out = 0;

            ElapsedTime timer2 = new ElapsedTime();

            //keeps the slider at the position and resistant to gravity
                    if (viperSlide2.getCurrentPosition() != l2) {

                        double error2 = l2 - viperSlide2.getCurrentPosition();

                        double derivative2 = (error2 - lastError2) / timer2.seconds();

                        integralSum2 = integralSum2 + (error2 * timer2.seconds());

                        double out2 = (p * error2) + (i * integralSum2) + (d * derivative2);

                        viperSlide2.setPower(out2);

                        lastError2 = error2;

                        timer2.reset();
                    }
    }
    
    
    public void moveForward(double n) {
        RobotLog.d("Front Left Possible Malfunction before moving forward; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before moving forward; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before moving forward; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before moving forward; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        moveVertical(-n);
        RobotLog.d("Front Left Possible Malfunction after moving forward; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after moving forward; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after moving forward; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after moving forward; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void moveBackward(double n) {
        RobotLog.d("Front Left Possible Malfunction before moving backward; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before moving backward; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before moving backward; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before moving backward; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        moveVertical(n);
        RobotLog.d("Front Left Possible Malfunction after moving backward; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after moving backward; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after moving backward; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after moving backward; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void moveViper(double n) {
        RobotLog.d("Slider Malfunction before moving slider; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        moveSlider(n);
        RobotLog.d("Slider Malfunction after moving slider; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void moveSlider(double n) {
        viperSlide.setPower(n);
        viperSlide2.setPower(-n);
    }
    
    public void moveClawRot(double n) {
        clawRot.setPower(n);
    }
    
    // If n is positive, move forward
    // If n is negative, move backward
    public void moveVertical(double n) {
        RobotLog.d("Front Left Possible Malfunction start of moveVertical; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction start of moveVertical; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction start of moveVertical; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction start of moveVertical; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        n = 1 * n;
        RobotLog.d("Front Left Possible Malfunction after setting n; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        frontLeft.setPower(0.75 * n);
        RobotLog.d("Front Left Possible Malfunction after setting power of FrontLeft; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after setting power of FrontLeft; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setPower(-0.75 * n);
        RobotLog.d("Front Right Possible Malfunction after setting power of FrontRight; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after setting power of FrontRight; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setPower(0.75 * n);
        RobotLog.d("Back Left Possible Malfunction after setting power of BackLeft; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after setting power of BackLeft; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setPower(-0.75 * n);
        RobotLog.d("Back Right Possible Malfunction after setting power of BackRight; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void moveLeft(double n) {
        RobotLog.d("Front Left Possible Malfunction before moving left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before moving left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before moving left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before moving left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        moveHorizontal(-n);
        RobotLog.d("Front Left Possible Malfunction after moving left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after moving left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after moving left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after moving left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void moveRight(double n) {
        RobotLog.d("Front Left Possible Malfunction before moving right; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before moving right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before moving right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before moving right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        moveHorizontal(n); 
        RobotLog.d("Front Left Possible Malfunction after moving right; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after moving right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after moving right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after moving right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }    
    
    // If n is positive, move left
    // If n is negative, move right
    public void moveHorizontal(double n) {
        RobotLog.d("Front Left Possible Malfunction at start of moveHorizontal; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        frontLeft.setPower(-0.5 * n);
        RobotLog.d("Front Left Possible Malfunction after setting power of front left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after setting power of front left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setPower(-0.5 * n);
        RobotLog.d("Front Right Possible Malfunction after setting power of front right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after setting power of front right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setPower(0.5 * n);
        RobotLog.d("Back Left Possible Malfunction after setting power of back left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after setting power of back left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setPower(0.5 * n); 
        RobotLog.d("Back Right Possible Malfunction after setting power of back right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }    
    
    public void rotateRight (double n) {
        RobotLog.d("Front Left Possible Malfunction before rotating right; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before rotating right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before rotating right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before rotating right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        rotate(n);
        RobotLog.d("Front Left Possible Malfunction after rotating right; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after rotating right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after rotating right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after rotating right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void rotateLeft (double n) {
        RobotLog.d("Front Left Possible Malfunction before rotating left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before rotating left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before rotating left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before rotating left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        rotate(-n);
        RobotLog.d("Front Left Possible Malfunction after rotating left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after rotating left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after rotating left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after rotating left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }    
    
    public void rotate (double n) {
        RobotLog.d("Front Left Possible Malfunction at the start of rotating; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction at the start of rotating; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction at the start of rotating; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction at the start of rotating; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        n = 1 * n;
        RobotLog.d("Front Left Possible Malfunction after n has been set; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        frontLeft.setPower(-0.4 * n);
        RobotLog.d("Front Left Possible Malfunction after setting power of front left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after setting power of front left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setPower(-0.4 * n);
        RobotLog.d("Front Right Possible Malfunction after setting power of front right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after setting power of front right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setPower(-0.4 * n);
        RobotLog.d("Back Left Possible Malfunction after setting power of back left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after setting power of back left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setPower(-0.4 * n);
        RobotLog.d("Back Right Possible Malfunction after setting power of back right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }    
    
    
    public void stopMovement() {
        RobotLog.d("Front Left Possible Malfunction at the start of stopping movement; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        frontLeft.setPower(0);
        RobotLog.d("Front Left Possible Malfunction after stopping front left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after stopping front left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setPower(0);
        RobotLog.d("Front Right Possible Malfunction after stopping front right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after stopping front right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setPower(0);
        RobotLog.d("Back Left Possible Malfunction after stopping back left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after stopping back left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setPower(0);
        RobotLog.d("Back Right Possible Malfunction after stopping back right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void stopSlider(){
        RobotLog.d("Slider Malfunction before stopping slider; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(0);
        viperSlide2.setPower(0);
        RobotLog.d("Slider Malfunction after stopping slider; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void stopClawRot() {
        clawRot.setPower(0);
    }

    public void moveClaw(double o) {
        claw.setPower(o);
        //if (claw.getPower() > 0.7 || claw.getPower() < -0.5) {
        //    throw new RuntimeException();
        //}
    }

    /*public void moveClaw(double o) {
        RobotLog.d("Claw Malfunction before setting power; Claw Curr Pos = %f", claw.getPower());
        claw.setPower(o);
        RobotLog.d("Claw Malfunction after setting power; Claw Curr Pos = %f", claw.getPower());
    }*/
    
    
    public void robotStop() { 
        RobotLog.d("Front Left Possible Malfunction before stopping robot; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        frontLeft.setPower(0);
        RobotLog.d("Front Left Possible Malfunction after frontLeft stops; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after frontLeft stops; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setPower(0);
        RobotLog.d("Front Right Possible Malfunction after frontRight stops; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after frontRight stops; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setPower(0);
        RobotLog.d("Back Left Possible Malfunction after backLeft stops; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after backLeft stops; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setPower(0);
        RobotLog.d("Back Right Possible Malfunction after backRight stops; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        RobotLog.d("Slider Malfunction before slide stops; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setPower(0);
        RobotLog.d("Slider Malfunction after slide stops; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
 
    
    public void stopCoast() {
        RobotLog.d("Front Left Possible Malfunction at the start of stopping coast; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotLog.d("Front Left Possible Malfunction after frontRight brakes; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after frontRight brakes; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotLog.d("Front Right Possible Malfunction after frontLeft brakes; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after frontLeft brakes; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);RobotLog.d("Front Left Possible Malfunction; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after backLeft brakes; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after backLeft brakes; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);RobotLog.d("Front Left Possible Malfunction; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after backRight brakes; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }

    //Moves the claw
   
    
    public void reverseMotors() {
        RobotLog.d("Front Left Possible Malfunction before reversing motor; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before reversing motor; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before reversing motor; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before reversing motor; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        RobotLog.d("Front Left Possible Malfunction after reversing frontRight; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        backRight.setDirection(DcMotor.Direction.REVERSE);
        RobotLog.d("Front Left Possible Malfunction after reversing backRight; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after reversing backRight; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
    }
    
    public void resetEncoders() {
        RobotLog.d("Front Left Possible Malfunction before resetting encoders; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before resetting encoders; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before resetting encoders; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before resetting encoders; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.d("Front Left Possible Malfunction after resetting frontRight encoder; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after resetting frontRight encoder; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.d("Front Left Possible Malfunction after resetting frontLeft encoder; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after resetting frontLeft encoder; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.d("Back Left Possible Malfunction after resetting backRight encoder; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after resetting backRight encoder; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.d("Back Left Possible Malfunction after resetting backLeft encoder; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Slider Malfunction before resetting slider encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotLog.d("Slider Malfunction after resetting slider encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void targetPosition(int n) {
        RobotLog.d("Front Left Possible Malfunction before setting position; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before setting position; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before setting position; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before setting position; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        frontLeft.setTargetPosition(n);
        RobotLog.d("Front Left Possible Malfunction after setting frontLeft position; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after setting frontLeft position; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setTargetPosition(n);
        RobotLog.d("Front Right Possible Malfunction after setting frontRight position; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after setting frontRight position; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setTargetPosition(n);
        RobotLog.d("Back Left Possible Malfunction after setting backLeft position; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after setting backLeft position; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setTargetPosition(n);
        RobotLog.d("Back Right Possible Malfunction after setting backRight position; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public void runPosition() {
        RobotLog.d("Front Left Possible Malfunction before running to position; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before running to position; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before running to position; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before running to position; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Front Left Possible Malfunction after frontLeft is running to position; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after frontLeft is running to position; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Front Right Possible Malfunction after frontRight is running to position; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after frontRight is running to position; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Back Left Possible Malfunction after backLeft is running to position; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after backLeft is running to position; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Back Right Possible Malfunction after backRight is running to position; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        RobotLog.d("Slider Malfunction before slider runs to position; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Slider Malfunction after slide runs to position; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }
    
    public void setPower(double n) {
        RobotLog.d("Front Left Possible Malfunction before setting power; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction before setting power; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction before setting power; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction before setting power; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        frontLeft.setPower(n);
        RobotLog.d("Front Left Possible Malfunction after setting power of front left; Front Left Curr Pos = %d", frontLeft.getCurrentPosition());
        RobotLog.d("Front Right Possible Malfunction after setting power of front left; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        frontRight.setPower(n);
        RobotLog.d("Front Right Possible Malfunction after setting power of front right; Front Right Curr Pos = %d", frontRight.getCurrentPosition());
        RobotLog.d("Back Left Possible Malfunction after setting power of front right; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        backLeft.setPower(n);
        RobotLog.d("Back Left Possible Malfunction after setting power of back left; Back Left Curr Pos = %d", backLeft.getCurrentPosition());
        RobotLog.d("Back Right Possible Malfunction after setting power of back left; Back Right Curr Pos = %d", backRight.getCurrentPosition());
        backRight.setPower(n);
        RobotLog.d("Back Right Possible Malfunction after setting power of back right; Back Right Curr Pos = %d", backRight.getCurrentPosition());
    }
    
    public CRServo getClaw() {
        return claw;
    }
    
    public DcMotor getClawRot() {
        return clawRot;
    }
    
    public DcMotor getSlide() {
        return viperSlide;
    }
    
    public DcMotor getSlide2() {
        return viperSlide2;
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
    

    
    /*public void Cycles (boolean cp, double c, int s, double l, double o, String rs, String cs, String crs, String ss, Telemetry telemetry) {
        //CyclesP1 (cp, c, s, l, o, telemetry);
        //CyclesP2 (cp, c, s, l, o, telemetry);
        //                  S1: close claw
        RobotLog.d("Claw malfunction before closing at start of cycle; Claw Curr Pos = %f", claw.getPower());
        cp = false;
        RobotLog.d("Claw Malfunction after setting clawPos to false; Claw Curr Pos = %f", claw.getPower());
        cs = "closing claw";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        moveClaw(c);
        RobotLog.d("Claw Malfunction after closing claw at start of cycle; Claw Curr Pos = %f", claw.getPower());
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            RobotLog.d("c", c);
        }
        //                  S2: move slider so it is right under the pole
        ss = "moving slider up to the pole";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving up; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        encoderSlide(s - 4200);
        RobotLog.d("Slider Malfunction after setting encoder to move up to pole; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        l = (s - 4200);
        RobotLog.d("Slider Malfunction after setting lastVsPos to s; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        holdViper(l);
        RobotLog.d("Slider Malfunction after holding the slide kit up at the pole; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        telemetry.addData("SlidePath",  "Running at %7d", viperSlide.getCurrentPosition());
        telemetry.update();
        RobotLog.d("Slider Malfunction after moving the slider up to the pole; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        //sleep(500);
        //                  S3: move clawRot to the back position
        crs = "moving to the back position";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("ClawRot Malfunction before moving to back position; ClawRot Curr Pos = %f", clawRot.getPower());
        moveClawRot(1);
        RobotLog.d("ClawRot Malfunction after moving to back position; ClawRot Curr Pos = %f", clawRot.getPower());
        try {
            Thread.sleep(1000); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S4: move slider down so cone is resting on the pole
        ss = "moving down to rest cone on pole";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving down so cone is resting on pole; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        if (clawRot.getPower() == 1) {
            RobotLog.d("Slider Malfunction after checking if clawRot is fully up; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
            encoderSlide(s - 3700);
            RobotLog.d("Slider Malfunction after setting slider encoders; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
            l = (s - 3700);
            RobotLog.d("Slider Malfunction after setting lastVSPos; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
            holdViper(l);
            RobotLog.d("Slider Malfunction after holding; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        RobotLog.d("Slider Malfunction after moving so cone is on the pole; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        // encoderSlide(s - 3700);
        // l = (s - 3700);
        // holdViper(l);
        try {
            Thread.sleep(750); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S5: open the claw and drop the cone
        RobotLog.d("Claw Malfunction before opening claw to drop cone on the pole; Claw Curr Pos = %f", claw.getPower());
        cp = true;
        RobotLog.d("Claw Malfunction after setting clawPos to true; Claw Curr Pos = %f", claw.getPower());
        cs = "opening claw to drop cone";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Claw Malfunction before opening claw; Claw Curr Pos = %f", claw.getPower());
        moveClaw(o);
        RobotLog.d("Claw Malfunction after opening claw; Claw Curr Pos = %f", claw.getPower());
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S6: move the slider up so that the claw can safely go to the front position
        ss = "moving slider up for clawRot";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving the slider back up; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        encoderSlide(s - 4200);
        RobotLog.d("Slider Malfunction after setting encoders; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        l = (s - 4200);
        RobotLog.d("Slider Malfunction after setting lastVSPos; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        holdViper(l);
        RobotLog.d("Slider Malfunction after holding the slide up; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        //                  S7: move the clawRot to the front position so that slider can go down safely
        crs = "moving clawRot to front position";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("ClawRot Malfunction before moving to front position to move down; ClawRot Curr Pos = %f", clawRot.getPower());
        moveClawRot(0.15);
        RobotLog.d("ClawRot Malfunction after moving to front position to move down; ClawRot Curr Pos = %f", clawRot.getPower());
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S8: move the slider down to pick up another cone
        ss = "moving slider down to pick up next cone";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving down to pick up the next cone; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        encoderSlide(s);
        RobotLog.d("Slider Malfunction after setting the encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        l = s;
        RobotLog.d("Slider Malfunction after setting lastVSPos; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        holdViper(l);
        RobotLog.d("Slider Malfunction after holding viper; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }

    public void CyclesP1 (boolean cp, double c, int s, double l, double o, String rs, String cs, String crs, String ss, Telemetry telemetry) {
        //                  S1: close claw
        RobotLog.d("Claw Malfunction before closing claw; Claw Curr Pos = %f", claw.getPower());
        cp = false;
        RobotLog.d("Claw Malfunction after setting to false; Claw Curr Pos = %f", claw.getPower());
        cs = "closing claw around cone";
        RobotLog.d("Claw Malfunction after state var; Claw Curr Pos = %f", claw.getPower());
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Claw Malfunction before closing; Claw Curr Pos = %f", claw.getPower());
        moveClaw(c);
        RobotLog.d("Claw Malfunction after closing claw; Claw Curr Pos = %f", claw.getPower());
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S2: move slider so it is right under the pole
        ss = "moving slider up above the pole";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction Cycles common error; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        encoderSlide(s - 4200);
        RobotLog.d("Slider Malfunction Cycles common error; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        l = (s - 4200);
        RobotLog.d("Slider Malfunction Cycles common error; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        holdViper(l);
        RobotLog.d("Slider Malfunction Cycles common error; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        telemetry.addData("SlidePath",  "Running at %7d", viperSlide.getCurrentPosition());
        telemetry.update();
        //sleep(500);
        //                  S3: move clawRot to the back position
        crs = "moving clawRot to back position";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("ClawRot Malfunction before moving to back pos; ClawRot Curr Pos = %f", clawRot.getPower());
        moveClawRot(1);
        RobotLog.d("ClawRot Malfunction after moving to back pos; ClawRot Curr Pos = %f", clawRot.getPower());
        try {
            Thread.sleep(1000); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        
    }

    public void CyclesP2 (boolean cp, double c, int s, double l, double o, String rs, String cs, String crs, String ss, Telemetry telemetry) {
        //                  S4: move slider down so cone is resting on the pole
        ss = "moving slider down so cone is on pole";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving down; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        if (clawRot.getPower() == 1) {
            RobotLog.d("Slider Malfunction before setting encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
            encoderSlide(s - 3700);
            RobotLog.d("Slider Malfunction after setting encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
            l = (s - 3700);
            RobotLog.d("Slider Malfunction after setting lastvspos; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
            holdViper(l);
            RobotLog.d("Slider Malfunction after holding viper; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        // encoderSlide(s - 3700);
        // l = (s - 3700);
        // holdViper(l);
        try {
            Thread.sleep(750); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S5: open the claw and drop the cone
        RobotLog.d("Claw Malfunction before opening claw; Claw Curr Pos = %f", claw.getPower());
        cp = true;
        RobotLog.d("Claw Malfunction after setting to true; Claw Curr Pos = %f", claw.getPower());
        cs = "open claw and drop cone";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Claw Malfunction after state var; Claw Curr Pos = %f", claw.getPower());
        moveClaw(o);
        RobotLog.d("Claw Malfunction after opening claw; Claw Curr Pos = %f", claw.getPower());
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S6: move the slider up so that the claw can safely go to the front position
        ss = "move slide back up so clawRot can move";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving back up; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        encoderSlide(s - 4200);
        RobotLog.d("Slider Malfunction after setting encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        l = (s - 4200);
        RobotLog.d("Slider Malfunction after setting lastvspos; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        holdViper(l);
        RobotLog.d("Slider Malfunction after holding; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        //                  S7: move the clawRot to the front position so that slider can go down safely
        crs = "move clawRot to the front position";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("ClawRot Malfunction before moving to frnt; ClawRot Curr Pos = %f", clawRot.getPower());
        moveClawRot(0.15);
        RobotLog.d("ClawRot Malfunction after moving to front; ClawRot Curr Pos = %f", clawRot.getPower());
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S8: move the slider down to pick up another cone
        ss = "move slider down to pick up another cone";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        RobotLog.d("Slider Malfunction before moving down; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        encoderSlide(s);
        RobotLog.d("Slider Malfunction after setting encoder; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        l = s;
        RobotLog.d("Slider Malfunction after setting lastvspos; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        holdViper(l);
        RobotLog.d("Slider Malfunction after holding; Viper Slide Curr Pos = %d", viperSlide.getCurrentPosition());
    }

    public void CyclesExceptions (boolean cp, double c, int s, double l, double o, String rs, String cs, String crs, String ss, Telemetry telemetry) {
        //CyclesP1 (cp, c, s, l, o, telemetry);
        //CyclesP2 (cp, c, s, l, o, telemetry);
        //                  S1: close claw
        cp = false;
        cs = "closing claw";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            moveClaw(c);
        } catch (RuntimeException e) {
            RobotLog.d("Claw EXCEPTION; Claw Curr Pos = %f", claw.getPower());
        }
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            RobotLog.d("c", c);
        }
        //                  S2: move slider so it is right under the pole
        ss = "moving slider up to the pole";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            encoderSlide(s - 4200);
        } catch (RuntimeException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            l = (s - 4200);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            holdViper(l);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        telemetry.addData("SlidePath",  "Running at %7d", viperSlide.getCurrentPosition());
        telemetry.update();
        //sleep(500);
        //                  S3: move clawRot to the back position
        crs = "moving to the back position";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            moveClawRot(1);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("ClawRot EXCEPTION; ClawRot Curr Pos = %f", clawRot.getPower());
        }
        try {
            Thread.sleep(1000); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S4: move slider down so cone is resting on the pole
        ss = "moving down to rest cone on pole";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        if (clawRot.getPower() == 1) {
            try {
            encoderSlide(s - 3700);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            l = (s - 3700);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            holdViper(l);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        }
        // encoderSlide(s - 3700);
        // l = (s - 3700);
        // holdViper(l);
        try {
            Thread.sleep(750); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S5: open the claw and drop the cone
        try {
            cp = true;
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Claw EXCEPTION; Claw Curr Pos = %f", claw.getPower());
        }
        cs = "opening claw to drop cone";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            moveClaw(o);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Claw EXCEPTION; Claw Curr Pos = %f", claw.getPower());
        }
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S6: move the slider up so that the claw can safely go to the front position
        ss = "moving slider up for clawRot";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            encoderSlide(s - 4200);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            l = (s - 4200);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            holdViper(l);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        //                  S7: move the clawRot to the front position so that slider can go down safely
        crs = "moving clawRot to front position";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            moveClawRot(0.15);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("ClawRot EXCEPTION; ClawRot Curr Pos = %f", clawRot.getPower());
        }
        try {
            Thread.sleep(500); // optional pause after each move
        } catch (InterruptedException e) {
            //Ignore exception
        }
        //                  S8: move the slider down to pick up another cone
        ss = "moving slider down to pick up next cone";
        //telemetry.addData("State of Robot:", robotState);
        //telemetry.update();
        try {
            encoderSlide(s);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            l = (s);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
        try {
            holdViper(l);
            Thread.sleep(0);
        } catch (InterruptedException e) {
            RobotLog.d("Slide EXCEPTION; Slide Curr Pos = %d", viperSlide.getCurrentPosition());
        }
    }*/
    
    
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    /*public void encoderDrive(
        boolean opModeIsActive,
        double speed, 
        double frontLeftInches, double frontRightInches, 
        double backLeftInches, double backRightInches, double timeoutS, Telemetry telemetry) 
    {
        // Ensure that the opmode is still active
        if (opModeIsActive) {
            
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
            
    
    }*/
    
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