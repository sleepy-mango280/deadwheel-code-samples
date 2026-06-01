@TeleOp(name = "Delta Test")
public class DeltaTest extends LinearOpMode {

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

        // remember the previous tick reading each cycle
        int lastParallel = 0;
        int lastPerp     = 0;

        // running total of distance traveled
        double totalForward = 0.0;
        double totalStrafe  = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            int currentParallel = parallelEncoder.getCurrentPosition();
            int currentPerp     = perpEncoder.getCurrentPosition();

            // how much did each encoder move THIS cycle?
            int deltaParallel = currentParallel - lastParallel;
            int deltaPerp     = currentPerp     - lastPerp;

            // convert deltas to inches
            double deltaForward = (deltaParallel / TICKS_PER_REV) * CIRCUMFERENCE;
            double deltaStrafe  = (deltaPerp     / TICKS_PER_REV) * CIRCUMFERENCE;

            // accumulate into running totals
            totalForward += deltaForward;
            totalStrafe  += deltaStrafe;

            // update previous readings for next cycle
            lastParallel = currentParallel;
            lastPerp     = currentPerp;

            telemetry.addData("Delta forward (in)", deltaForward);
            telemetry.addData("Delta strafe  (in)", deltaStrafe);
            telemetry.addData("Total forward (in)", totalForward);
            telemetry.addData("Total strafe  (in)", totalStrafe);
            telemetry.update();
        }
    }
}

