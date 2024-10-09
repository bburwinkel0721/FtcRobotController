//2324TeleOpWLift
package org.firstinspires.ftc.teamcode;

// Program Notes: This is a simple TeleOP program for mecanum wheel drive with a field centric view
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp2425WLift", group="Linear Opmode")

public class TeleOp2425WLift extends LinearOpMode {

    // Use this method as a form of print statement for the robot, telemetry messages will appear in the bottom left corner of the driver station screen
    public void telemetryUpdate(String input1, String input2) {
        telemetry.addData(input1, input2);
        telemetry.update();
    }

    public void telemetryAdd(String input1, String input2) {
        telemetry.addLine()
        .addData(input1, input2);
        telemetry.update();
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf_Motor;
    private DcMotor lr_Motor;
    private DcMotor rf_Motor;
    private DcMotor rr_Motor;
    
    private DcMotor liftMotor;
    private DcMotor Crunches;


    /* private Servo claw1;
    private Servo claw2; */ 
    
    // private CRServo spinny;

    public void runOpMode() {

        telemetryUpdate("Status", "Initialized");

        // Map hardware

        // Retrieve the IMU from the hardware map, this is from a guide on Mecanum drive, this can be used as a gyro sensor
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        lf_Motor = hardwareMap.dcMotor.get("left_motorFront");
        lr_Motor = hardwareMap.dcMotor.get("left_motorRear");
        rf_Motor = hardwareMap.dcMotor.get("right_motorFront");
        rr_Motor = hardwareMap.dcMotor.get("right_motorRear");
        Crunches = hardwareMap.dcMotor.get("Crunches");


       /* claw1 = hardwareMap.get(Servo.class, "claw_servo1");
        claw2 = hardwareMap.get(Servo.class, "claw_servo2"); */
        
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
       //spinny = hardwareMap.crservo.get("spinny");
        //Reverse right side motors
        rf_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rr_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetryUpdate("Status", "Waiting for start");

        waitForStart();
        runtime.reset();

        telemetryUpdate("Status", "Robot started");

        if (isStopRequested()) return;

            while (opModeIsActive()) {

                // Setup variables for gamepad inputs
                double y = gamepad1.left_stick_y; // Remember, this is reversed
                double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = -gamepad1.right_stick_x;

                // Read inverse IMU heading, as the IMU heading is CW positive
                double botHeading = -imu.getAngularOrientation().firstAngle; // The robot currently does not drive in the correct direction after turning, this vlaue may need to be inversed

                double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double lf_Power = (rotY + rotX + rx) / denominator;
                double lr_Power = (rotY - rotX + rx) / denominator;
                double rf_Power = (rotY - rotX - rx) / denominator;
                double rr_Power = (rotY + rotX - rx) / denominator;

                // Half power
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    lf_Power = lf_Power / 2;
                    rf_Power = rf_Power / 2;
                    lr_Power = lr_Power / 2;
                    rr_Power = rr_Power / 2;
                }

                // Send calculated power to motors
                lf_Motor.setPower(lf_Power);
                lr_Motor.setPower(lr_Power);
                rf_Motor.setPower(rf_Power);
                rr_Motor.setPower(rr_Power);

                // Arm control
                double liftPower = gamepad2.left_stick_y;
                liftMotor.setPower(liftPower/2);
                if (gamepad2.b) {
                    liftMotor.setPower(.25);
                }

                // Claw control
                /* if (gamepad2.x) {
                   claw1.setPosition(0.4);
                   claw2.setPosition(0.7);
                } else if (gamepad2.y) {
                   claw1.setPosition(0.5);
                   claw2.setPosition(0.4); */
                }
                //make the robot do crunches for lifting up
                //DcMotorCrunches.setPower();
               if (gamepad2.x) {
                    Crunches.setPower(.75);
                } else if (gamepad2.y) {
                   Crunches.setPower(0);
                }
                
            
            /*   claw1  = hardwareMap.get(Servo.class, "claw1");
                claw2 = hardwareMap.get(Servo.class, "claw2"); */
             /* Reset imu if (driver 1 presses x);
               if (gamepad1.x) {
                    Retrieve the IMU from the hardware map, this is from a guide on Mecanum drive, this can be used as a gyro sensor
                    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
                    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                     Technically this is the default, however specifying it is clearer
                    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                     Without this, data retrieving from the IMU throws an exception
                   imu.initialize(parameters);
               } */ 
             }
    }

