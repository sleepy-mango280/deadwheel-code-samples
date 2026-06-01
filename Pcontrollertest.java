import com.acmerobotics.dashboard.config.Config;

@Config 
@TeleOp(name = "P Controller Test")
public class PControllerTest extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor parallelEncoder;

    static final double TICKS_PER_REV  = 2000;
    static final double WHEEL_DIAMETER = 1.378;
    static final double CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER;

    public static double kP     = 0.05;  // start small, tune later
    public static double TARGET = 24.0;  // drive 24 inches forward

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        parallelEncoder = hardwareMap.get(DcMotor.class, "parallel");
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // where are we?
            int ticks = parallelEncoder.getCurrentPosition();
            double current = (ticks / TICKS_PER_REV) * CIRCUMFERENCE;

            // how far off are we?
            double error = TARGET - current;

            // how hard do we drive?
            double power = error * kP;

            // send power to all four motors
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Current (in)", current);
            telemetry.addData("Error (in)",   error);
            telemetry.addData("Power",         power);
            telemetry.update();
        }
    }
}
