package org.firstinspires.ftc.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
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
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;


@TeleOp
public class Test extends LinearOpMode {
    //create variables for the joysticks
    double rsy1 = 0;
    double rsx1 = 0;
    double lsy1 = 0;
    double lsx1 = 0;
    double rsy2 = 0;
    double rsx2 = 0;
    double lsy2 = 0;
    double lsx2 = 0;

    @Override
    public void runOpMode() {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        //wait till start is pressed on the driver hub
        waitForStart();
        
        double close = -1;
        double open = -0.3;
        robot.moveClaw(close);

        while (opModeIsActive()) {
            double rsy1 = this.gamepad1.right_stick_y;
            double rsx1 = this.gamepad1.right_stick_x;
            double lsy1 = this.gamepad1.left_stick_y;
            double lsx1 = this.gamepad1.left_stick_x;
            double rsy2 = this.gamepad2.right_stick_y;
            double rsx2 = this.gamepad2.right_stick_x;
            double lsy2 = this.gamepad2.left_stick_y;
            double lsx2 = this.gamepad2.left_stick_x;

            //when the joystick is up, it is at -1, and when it is down, it is at +1
            //when the joystick is right, it is at +1, and when it is left, it is at -1
            if (rsy1 != 0) {
                    robot.moveVertical(rsy1);
                } else if (rsx1 != 0) {
                    robot.rotate(rsx1);
                } else {
                    robot.stopMovement();
                }
                
            
            while (gamepad1.dpad_right) {
                robot.moveHorizontal(1);
            } 
            while (gamepad1.dpad_left) {
                robot.moveHorizontal(-1);
            }
            
            if (lsy1 > 0) {
                
                robot.moveSlider(lsy1*0.6);
                
            } else if (lsy1 < 0) {
                
                robot.moveSlider(lsy1);
                
            } else {
                //robot.stopSlider();
                
                robot.moveSlider(-0.0005);
                
                /*if (onceSlide == false) {
                    robot.moveSlider(holdingSlide);
                    onceSlide = true;
                }*/
                
            }
            
            if (robot.viperSlide.getCurrentPosition() < 0) {
                robot.encoderSlide(100);
            }
            
            if (robot.viperSlide.getCurrentPosition() > 4000) {
                robot.encoderSlide(3900);
            }
            
            if (robot.getClaw().getPower() < (close+0.01) && robot.getClaw().getPower() > (close-0.01) && gamepad1.right_bumper) {
                robot.moveClaw(open);
                sleep(200);
            } else if (robot.getClaw().getPower() < (open+0.01) && robot.getClaw().getPower() > (open-0.01) && gamepad1.right_bumper) {
                robot.moveClaw(close);
                sleep(200);
            }

            telemetry.addData("Right Stick Y 1", rsy1);
            telemetry.addData("Left Stick Y 1", lsy1);
            telemetry.addData("Left Stick X 1", lsx1);
            telemetry.addData("Right Stick X 1", rsx1);
            telemetry.addData("Right Stick Y 2", rsy2);
            telemetry.addData("Left Stick Y 2", lsy2);
            telemetry.addData("Left Stick X 2", lsx2);
            telemetry.addData("Right Stick X 2", rsx2);
            telemetry.addData("Power of frontLeft", robot.frontLeft.getPower());
            telemetry.addData("Power of backLeft", robot.backLeft.getPower());
            telemetry.addData("Power of frontRight", robot.frontRight.getPower());
            telemetry.addData("Power of backRight", robot.backRight.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }

    }

}