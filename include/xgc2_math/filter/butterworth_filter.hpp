#ifndef XGC2_MATH_BUTTERWORTH_FILTER_HPP
#define XGC2_MATH_BUTTERWORTH_FILTER_HPP

#include <algorithm>
#include <cmath>

namespace xgc2_math {

class SecondOrderButterworthLowPass {
  public:
    SecondOrderButterworthLowPass() = default;

    explicit SecondOrderButterworthLowPass(double cutoff_frequency_hz, double initial_value = 0.0) {
        reset(cutoff_frequency_hz, initial_value);
    }

    void reset(double cutoff_frequency_hz, double initial_value = 0.0) {
        cutoff_frequency_hz_ = cutoff_frequency_hz;
        resetState(initial_value);
    }

    void resetState(double value = 0.0) {
        x1_ = value;
        x2_ = value;
        y1_ = value;
        y2_ = value;
        initialized_ = true;
    }

    double filter(double input, double dt_s) {
        if (!std::isfinite(input)) {
            return y1_;
        }

        if (!initialized_) {
            reset(cutoff_frequency_hz_, input);
            return input;
        }

        if (cutoff_frequency_hz_ <= 0.0 || dt_s <= 0.0 || !std::isfinite(dt_s)) {
            resetState(input);
            return input;
        }

        const Coefficients c = coefficients(cutoff_frequency_hz_, dt_s);
        const double output = c.b0 * input + c.b1 * x1_ + c.b2 * x2_ - c.a1 * y1_ - c.a2 * y2_;
        x2_ = x1_;
        x1_ = input;
        y2_ = y1_;
        y1_ = output;
        return output;
    }

    double value() const { return y1_; }

    double cutoffFrequencyHz() const { return cutoff_frequency_hz_; }

    bool initialized() const { return initialized_; }

  private:
    struct Coefficients {
        double b0{1.0};
        double b1{0.0};
        double b2{0.0};
        double a1{0.0};
        double a2{0.0};
    };

    static Coefficients coefficients(double cutoff_frequency_hz, double dt_s) {
        constexpr double kSqrt2 = 1.4142135623730950488;
        constexpr double kFilterPi = 3.14159265358979323846;

        const double sample_frequency_hz = 1.0 / dt_s;
        const double nyquist_hz = 0.5 * sample_frequency_hz;
        const double cutoff_hz = std::max(1.0e-9, std::min(cutoff_frequency_hz, 0.45 * nyquist_hz));
        const double warped = std::tan(kFilterPi * cutoff_hz / sample_frequency_hz);
        const double warped2 = warped * warped;
        const double norm = 1.0 / (1.0 + kSqrt2 * warped + warped2);

        Coefficients c;
        c.b0 = warped2 * norm;
        c.b1 = 2.0 * c.b0;
        c.b2 = c.b0;
        c.a1 = 2.0 * (warped2 - 1.0) * norm;
        c.a2 = (1.0 - kSqrt2 * warped + warped2) * norm;
        return c;
    }

    double cutoff_frequency_hz_{0.0};
    double x1_{0.0};
    double x2_{0.0};
    double y1_{0.0};
    double y2_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_BUTTERWORTH_FILTER_HPP
