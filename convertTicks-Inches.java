@TeleOp(name = "Two Encoder Test")
public class TwoEncoderTest extends LinearOpMode {

    DcMotor parallelEncoder;
    DcMotor perpEncoder;

    static final double TICKS_PER_REV  = 2000;
    static final double WHEEL_DIAMETER = 1.378;
    static final double CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER;

    @Override
    public void runOpMode() {

        parallelEncoder = hardwareMap.get(DcMotor.class, "parallel");
        perpEncoder     = hardwareMap.get(DcMotor.class, "perp");

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            int parallelTicks = parallelEncoder.getCurrentPosition();
            int perpTicks     = perpEncoder.getCurrentPosition();

            double forward = (parallelTicks / TICKS_PER_REV) * CIRCUMFERENCE;
            double strafe  = (perpTicks     / TICKS_PER_REV) * CIRCUMFERENCE;

            telemetry.addData("Parallel ticks",  parallelTicks);
            telemetry.addData("Perp ticks",      perpTicks);
            telemetry.addData("Forward (inches)", forward);
            telemetry.addData("Strafe (inches)",  strafe);
            telemetry.update();
        }
    }
}
