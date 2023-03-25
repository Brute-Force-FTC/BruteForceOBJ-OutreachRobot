package org.firstinspires.ftc.bruteforce.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Autonomous program starting at Blue Storage location
 */
@Autonomous(name="AutoConePark", group="group1")
public class AutoConePark extends LinearOpMode {
    
    double                  globalAngle, power = .30, correction;
    
    @Override
    public void runOpMode() {
        
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        robot.moveClaw(-1);
        double lastVSpos = 0;
        double lastVSpos2 = 0;
        
        TensorFlowUtils utils = new TensorFlowUtils(hardwareMap);
        
        // Send telemetry message to signify robot waiting;
        String robotState = "idle";
        String slideState = "idle";
        String clawState = "idle";
        String clawRotState = "idle";
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.addData("State of Robot: ", robotState);
        telemetry.addData("State of Slider: ", slideState);
        telemetry.addData("State of Claw: ", clawState);
        telemetry.addData("State of ClawRot: ", clawRotState);
        telemetry.update();
    
        // make a method for resetting and running encoders
        robot.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d :%7d", robot.getFrontLeft().getCurrentPosition(), robot.getFrontRight().getCurrentPosition(), robot.getBackLeft().getCurrentPosition(), robot.getBackRight().getCurrentPosition(), robot.getSlide().getCurrentPosition());
        telemetry.update();
        
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        // Wait for the game to start (driver presses PLAY)
        // Wait for TensorFlow to initialize
        sleep(5000);
        telemetry.addData(">****************", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        int sleeve = utils.returnTSEPosition(opModeIsActive(), telemetry);
        telemetry.addData("Sleeve = ", sleeve);
        telemetry.update();
        
        
        int start = 0;
        int startSlide1 = robot.viperSlide.getCurrentPosition();
        int startSlide2 = robot.viperSlide2.getCurrentPosition();
        int startRot = robot.clawRot.getCurrentPosition();
        telemetry.addData("Start of Slide 1&2 and ClawRot:", "Starting at %7d :%7d :%7d", startSlide1, startSlide2, startRot);
        telemetry.update();
        
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        // the variable for Right Stick X, Right Stick Y, Left Stick X, Left Stick Y are defined
        
        //new
        
        double close = -0.7;
        double open = 0.2;
        
        robot.moveClaw(close);
        sleep(500);
        
        boolean flipped = false;
        double holdingRot = 0.0005;
        double holdingSlide = -0.0005;
        
        robot.resetAngle(globalAngle);
        
        correction = robot.checkDirectionB(globalAngle, telemetry);
        
        //move clawRot up
        robot.encoderRot(startRot + 500);
        robot.moveClawRot(holdingRot);
        
        //move robot backward to the third tile
        robot.encoderDriveIMU("b", 55, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move robot forward to the third tile
        robot.encoderDriveIMU("f", 11, globalAngle, 5, opModeIsActive(), telemetry);
        
        //turn right
        robot.encoderDriveIMU("rr", 14.5, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move clawRot up
        robot.encoderRot(startRot + 2000);
        robot.moveClawRot(holdingRot);
        
        //move robot backward
        robot.encoderDriveIMU("f", 3.25, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move clawRot up
        robot.encoderRot(startRot + 2200);
        robot.moveClawRot(holdingRot);
        sleep(500);
        
        //open claw
        robot.moveClaw(open);
        sleep(500);
        
        //move clawRot back down
        robot.encoderRot(startRot + 475);
        robot.moveClawRot(holdingRot);
        
        //move backwards
        robot.encoderDriveIMU("b", 12, globalAngle, 5, opModeIsActive(), telemetry);
        
        //turn right
        robot.encoderDriveIMU("rr", 4, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move backwards
        robot.encoderDriveIMU("b", 11.5, globalAngle, 5, opModeIsActive(), telemetry);
        
        //close claw
        robot.moveClaw(close);
        sleep(500);
        
        //move clawRot up
        robot.encoderRot(startRot + 1000);
        robot.moveClawRot(holdingRot);
        
        //move backwards
        robot.encoderDriveIMU("f", 10, globalAngle, 5, opModeIsActive(), telemetry);
        
        //turn right
        robot.encoderDriveIMU("rl", 4, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move backwards
        robot.encoderDriveIMU("f", 10, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move clawRot up
        robot.encoderRot(startRot + 2200);
        robot.moveClawRot(holdingRot);
        sleep(500);
        
        //open claw
        robot.moveClaw(open);
        sleep(500);
        
        //move clawRot down
        robot.encoderRot(startRot);
        robot.moveClawRot(holdingRot);
        sleep(500);
        
        //move backward
        robot.encoderDriveIMU("b", 3, globalAngle, 5, opModeIsActive(), telemetry);
        
        //turn left
        robot.encoderDriveIMU("rl", 14.5, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move forward
        robot.encoderDriveIMU("f", 18, globalAngle, 5, opModeIsActive(), telemetry);
        
        //strafe into the correct parking based on the sleeve
        if (sleeve==1) {
            
            //strafe left to park in spot 1
            robotState = "strafing left";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("l", 27, 5, opModeIsActive(), telemetry);
            
            //move robot forward a little for safety
            robotState = "moving forwards";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("b", 5, 5, opModeIsActive(), telemetry);
            
        } else if (sleeve==2) {
            
            //stay where you are for spot 2
            robotState = "idle";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update()
          ;
        } else if (sleeve==3) {
            
            //strafe right to park in spot 3
            robotState = "strafing right";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("r", 28, 5, opModeIsActive(), telemetry);
            
            //move forward a little for safety
            robotState = "moving forward";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("b", 5, 5, opModeIsActive(), telemetry);
            
        }
        
        
        /* 
        copy paste this for multiple cones
        
        //move clawRot back down
        robot.encoderRot(startRot + 450);
        robot.moveClawRot(holdingRot);
        
        //move backwards
        robot.encoderDriveIMU("b", 12, globalAngle, 5, opModeIsActive(), telemetry);
        
        //turn right
        robot.encoderDriveIMU("rr", 4, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move backwards
        robot.encoderDriveIMU("b", 10, globalAngle, 5, opModeIsActive(), telemetry);
        
        //close claw
        robot.moveClaw(close);
        sleep(500);
        
        //move clawRot up
        robot.encoderRot(startRot + 1000);
        robot.moveClawRot(holdingRot);
        
        //move backwards
        robot.encoderDriveIMU("f", 10, globalAngle, 5, opModeIsActive(), telemetry);
        
        //turn right
        robot.encoderDriveIMU("rl", 4, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move backwards
        robot.encoderDriveIMU("f", 10, globalAngle, 5, opModeIsActive(), telemetry);
        
        //move clawRot up
        robot.encoderRot(startRot + 2200);
        robot.moveClawRot(holdingRot);
        sleep(500);
        
        //open claw
        robot.moveClaw(open);
        sleep(500);
        
        */
        
        
        
        
        /*strafe into the correct parking based on the sleeve
        if (sleeve==1) {
            
            //strafe left to park in spot 1
            robotState = "strafing left";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("l", 27, 5, opModeIsActive(), telemetry);
            
            //move robot forward a little for safety
            robotState = "moving forwards";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("f", 5, 5, opModeIsActive(), telemetry);
            
        } else if (sleeve==2) {
            
            //stay where you are for spot 2
            robotState = "idle";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update()
          ;
        } else if (sleeve==3) {
            
            //strafe right to park in spot 3
            robotState = "strafing right";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("r", 28, 5, opModeIsActive(), telemetry);
            
            //move forward a little for safety
            robotState = "moving forward";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("f", 5, 5, opModeIsActive(), telemetry);
            
        }*/
        
        
        
        
        // Stop the robot
        robotState = "idle";
        slideState = "idle";
        clawRotState = "idle";
        clawState = "idle";
        robot.robotStop();
        
        telemetry.addData("Path", "Complete");
        telemetry.addData("State of Robot: ", robotState);
        telemetry.addData("State of Slider: ", slideState);
        telemetry.addData("State of Claw: ", clawState);
        telemetry.addData("State of ClawRot: ", clawRotState);
        telemetry.update();
    }
}
