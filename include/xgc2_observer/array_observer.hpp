#ifndef XGC2_OBSERVER_ARRAY_OBSERVER_HPP
#define XGC2_OBSERVER_ARRAY_OBSERVER_HPP

#include <array>
#include <cstddef>

#include "xgc2_observer/differentiator.hpp"
#include "xgc2_observer/luenberger_observer.hpp"

namespace xgc2_observer {

template <std::size_t N>
class ArrayDifferentiator {
public:
    ArrayDifferentiator() = default;

    explicit ArrayDifferentiator(DifferentiatorOptions options)
    {
        setOptions(options);
    }

    void setOptions(DifferentiatorOptions options)
    {
        for (auto& differentiator : differentiators_) {
            differentiator.setOptions(options);
        }
    }

    void reset()
    {
        for (auto& differentiator : differentiators_) {
            differentiator.reset();
        }
    }

    void reset(const std::array<double, N>& values)
    {
        for (std::size_t i = 0; i < N; ++i) {
            differentiators_[i].reset(values[i], 0.0);
        }
    }

    std::array<DifferentiatorSample, N> update(const std::array<double, N>& values, double dt_s)
    {
        std::array<DifferentiatorSample, N> samples{};
        for (std::size_t i = 0; i < N; ++i) {
            samples[i] = differentiators_[i].update(values[i], dt_s);
        }
        return samples;
    }

    std::array<double, N> values() const
    {
        std::array<double, N> values{};
        for (std::size_t i = 0; i < N; ++i) {
            values[i] = differentiators_[i].value();
        }
        return values;
    }

    std::array<double, N> derivatives() const
    {
        std::array<double, N> derivatives{};
        for (std::size_t i = 0; i < N; ++i) {
            derivatives[i] = differentiators_[i].derivative();
        }
        return derivatives;
    }

    const Differentiator& axis(std::size_t index) const
    {
        return differentiators_[index];
    }

    Differentiator& axis(std::size_t index)
    {
        return differentiators_[index];
    }

private:
    std::array<Differentiator, N> differentiators_{};
};

template <std::size_t N>
class ArrayPositionVelocityLuenbergerObserver {
public:
    ArrayPositionVelocityLuenbergerObserver() = default;

    explicit ArrayPositionVelocityLuenbergerObserver(PositionVelocityObserverOptions options)
    {
        setOptions(options);
    }

    void setOptions(PositionVelocityObserverOptions options)
    {
        for (auto& observer : observers_) {
            observer.setOptions(options);
        }
    }

    void reset()
    {
        for (auto& observer : observers_) {
            observer.reset();
        }
    }

    void reset(const std::array<double, N>& positions)
    {
        for (std::size_t i = 0; i < N; ++i) {
            observers_[i].reset(positions[i], 0.0);
        }
    }

    std::array<PositionVelocityEstimate, N> update(const std::array<double, N>& positions, double dt_s)
    {
        std::array<PositionVelocityEstimate, N> estimates{};
        for (std::size_t i = 0; i < N; ++i) {
            estimates[i] = observers_[i].update(positions[i], dt_s);
        }
        return estimates;
    }

    std::array<PositionVelocityEstimate, N> update(
        const std::array<double, N>& positions,
        double dt_s,
        const std::array<double, N>& accelerations)
    {
        std::array<PositionVelocityEstimate, N> estimates{};
        for (std::size_t i = 0; i < N; ++i) {
            estimates[i] = observers_[i].update(positions[i], dt_s, accelerations[i]);
        }
        return estimates;
    }

    std::array<double, N> positions() const
    {
        std::array<double, N> positions{};
        for (std::size_t i = 0; i < N; ++i) {
            positions[i] = observers_[i].position();
        }
        return positions;
    }

    std::array<double, N> velocities() const
    {
        std::array<double, N> velocities{};
        for (std::size_t i = 0; i < N; ++i) {
            velocities[i] = observers_[i].velocity();
        }
        return velocities;
    }

    const PositionVelocityLuenbergerObserver& axis(std::size_t index) const
    {
        return observers_[index];
    }

    PositionVelocityLuenbergerObserver& axis(std::size_t index)
    {
        return observers_[index];
    }

private:
    std::array<PositionVelocityLuenbergerObserver, N> observers_{};
};

}  // namespace xgc2_observer

#endif  // XGC2_OBSERVER_ARRAY_OBSERVER_HPP
