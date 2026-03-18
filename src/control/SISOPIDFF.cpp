#define _USE_MATH_DEFINES

#include <cmath>
#include "control/SISOPIDFF.hpp"
#include "math/math_util.hpp"

using namespace liteaerosim::control;

float SISOPIDFF::step(float cmdIn, float measIn, float measDotIn)
{

    float propPrev = prop();
    float ffPrev = feedfwd();
    float derivPrev = deriv();

    // optional cmd/meas unwrapping
    if (unwrapInputs) {
        measUnwrap.step(measIn);
        cmdUnwrap.step(cmdIn);
    } else {
        measUnwrap.reset(measIn);
        cmdUnwrap.reset(cmdIn);
    }

    cmdSignal.step(cmdUnwrap.out());
    measSignal.step(measUnwrap.out());

    float errorUnfiltered = cmdSignal.out() - measSignal.out();

    if (unwrapInputs) {

        errorUnfiltered = MathUtil::wrapToPi(errorUnfiltered);

        // We need to support the following for angular coordinates:
        //
        // Error signal may need to be clipped to a value bound (e.g. +/- pi/4)
        //
        // Error signal may need to be rate limited and filtered, but also the command
        // may cycle past the pi boundary and appear on the other side.  When that occurs,
        // we do *not* want the rate limit and filter to slew the error through 0 but
        // rather we want it to snap to the opposite limit.
        //
        // However, while the error signal is larger than the value limits, we do
        // not want to filter and rate limiter states to wind up past the value 
        // limits, because doing so would cause the filtered value to lag coming 
        // off of the limits.
        //
        // Solution: detect wrap crossing events and handle those separately.
        // We want to reset errSignal when that occurs.
        // We will see the one of the following conditions on a reset event:
        // 1) wrapToPi(errorUnfiltered - errorUnfiltered_prev) is opposite sign from
        //    wrapToPi(errorUnfiltered)
        //    In this case, the unfiltered value on its own is observed going
        //    past the pi boundary.
        // 
        // 2) wrapToPi(errorUnfiltered - errSignal.out()) is opposite sign from
        //    wrapToPi(errorUnfiltered)
        //    AND (both errorUnfiltered and errSignal.out() are nonzero
        //    AND errorUnfiltered and errSignal.out() are opposite sign from each other
        //    OR their wrapped difference is zero (i.e both are at the pi boundary))
        //    In this case, the unfiltered value is closer to the filtered value
        //    through the pi boundary than it is within the limited range.
        //
        // Probably detecting #2 is the only condition test needed.
        // In either case, the behavior we want to perform is to reset the
        // filtered value to the nearest value limit to the unfiltered value.
        // TODO: should we then step the filter, or should we wait until the
        // next iteration?

        if ((fabs(errorUnfiltered) > 0.0f)
            && (fabs(errSignal.out()) > 0.0f)
            && errorUnfiltered * MathUtil::wrapToPi(errSignal.out()) < 0.0f
            && (errorUnfiltered * MathUtil::wrapToPi(errorUnfiltered - errSignal.out()) <= 0.0f)) {

            // The error inputs have crossed the pi boundary, reset the filter to close 
            // to the nearest pi limit

            float piBoundNearest = (errorUnfiltered<0.0f) ? -M_PI : M_PI;

            // NOTE: one effect of this reset is that the rate limit and filtering
            // may have a jump as we cross the boundary due to loss of the information
            // of the previous error value as we jump to the nearest limit.  The size of the jump
            // will be proportional to how fast the unlimited error values are changing.
            // We will try to maintain that information by allowing the reset value to exceed
            // the pi boundary, but the value limit if enabled won't allow that.
            // In other words, say the previous filtered error value was pi - 0.01 
            // but the unfiltered error value crossed to -pi + 0.05.  Then we want to reset the 
            // error filter to -pi - 0.01.  The value of piBoundNearest will be -pi.
            //
            // If on the other hand the value limit is enabled at +/- pi/4, the previous filtered
            // error was pi/4 - 0.01 and the unfiltered error value has crossed to
            // -pi + 0.05.  Then piBoundNearest = -pi and we will reset to 
            // -pi + wrapToPi(pi/4-0.01 - - pi) = -pi -3*pi/4 - 0.01
            // but the value limit will clip the errSignal output (and state) to -pi/4.

            errSignal.resetToInput(piBoundNearest + MathUtil::wrapToPi(errSignal.out() - piBoundNearest));
        }

        errSignal.step(errorUnfiltered);

        // feed forward should probably only be used for cartesian input coordinates
        ffwdSignal.resetToInput(0.0f);

    } else {
        errSignal.step(errorUnfiltered);

        // feed forward should probably only be used for cartesian input coordinates
        ffwdSignal.step(cmdUnwrap.out());
    }

    // backsolve
    // TODO: make this optional
    // TODO: is there state where we should enable/disable backsolving?
    // For a backsolving implementation we only want to change the
    // integrator state based on SISO saturation of the 
    // integrator's output that occurs immediately downstream of the
    // output of this class.  We do not want to feed back any
    // filter dynamics or other static mappings.  Saturation information that
    // cannot be returned exactly risks causing integrator windup --
    // the very phenomenon that backsolving portends to mitigate.
    // If the saturation cannot be returned exactly, then it is safer
    // (and still reasonably performant) to use an antiwindup 
    // detection mode and freeze the integrator rather than attempting 
    // to correct it through backsolving.
    // TODO: investigate a method that applies a small signal deadzone
    // to suppress inexact backsolving while still performing backsolving
    // on large errors.
    // TODO: investigate dynamic inversion to approximately recover
    // the unfiltered version of a signal for use in backsolving
    // calculations.

    I.reset(out() - ffPrev - propPrev - derivPrev);

    I.step(Ki*errSignal.out());

    measDotSignal.step(measDotIn);

    outSignal.step( feedfwd() + prop() + integ() + deriv() );

    return out();
}

float SISOPIDFF::step(float cmdIn, float measIn)
{
    return step(cmdIn, measIn, D.step(measIn));
}

void SISOPIDFF::reset(float cmdIn, float measIn, float measDotIn, float outIn)
{
    // optional cmd/meas unwrapping
    if (unwrapInputs) {
        measUnwrap.step(measIn);
        cmdUnwrap.step(cmdIn, measUnwrap.out());
    } else {
        measUnwrap.reset(measIn);
        cmdUnwrap.reset(cmdIn);
    }

    cmdSignal.resetToInput(cmdUnwrap.out());
    measSignal.resetToInput(measUnwrap.out());
    errSignal.resetToInput(cmdSignal.out() - measSignal.out());
    ffwdSignal.resetToInput(cmdUnwrap.out());

    D.reset(measIn, measDotIn);

    measDotSignal.resetToInput(D.out());

    outSignal.resetToOutput( outIn );

    I.reset(outIn - (feedfwd() + prop() + deriv()));

}

void SISOPIDFF::reset(float cmdIn, float measIn, float outIn)
{
    reset(cmdIn, measIn, outIn, 0.0f);
}
