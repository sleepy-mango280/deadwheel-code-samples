@Config
@TeleOp(name = "PD Controller Test")
public class PDControllerTest extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor parallelEncoder;

    static final double TICKS_PER_REV  = 2000;
    static final double WHEEL_DIAMETER = 1.378;
    static final double CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER;

    public static double kP     = 0.05;
    public static double kD     = 0.005;
    public static double TARGET = 24.0;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        parallelEncoder = hardwareMap.get(DcMotor.class, "parallel");
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lastError = 0.0;   // memory for D term

        waitForStart();

        while (opModeIsActive()) {

            // step 1 -- where are we
            int ticks      = parallelEncoder.getCurrentPosition();
            double current = (ticks / TICKS_PER_REV) * CIRCUMFERENCE;

            // step 2 -- how far off are we
            double error = TARGET - current;

            // step 3 -- P term
            double pTerm = error * kP;

            // step 4 -- D term
            double deltaError = error - lastError;
            double dTerm      = deltaError * kD;

            // step 5 -- combine
            double power = pTerm + dTerm;

            // step 6 -- send to motors
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            // step 7 -- update memory
            lastError = error;

            telemetry.addData("Current (in)", current);
            telemetry.addData("Error (in)",   error);
            telemetry.addData("pTerm",         pTerm);
            telemetry.addData("dTerm",         dTerm);
            telemetry.addData("Power",         power);
            telemetry.update();
        }
    }
}
