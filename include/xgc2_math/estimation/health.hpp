#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace xgc2_math {

enum class VrpnObservationState {
    kTrusted,
    kSuspected,
    kFault,
    kRecovery,
};

enum class FilterHealth {
    kNominal,
    kDegraded,
    kImuOnly,
    kLost,
};

enum class PoseFusionRejectReason {
    kNone,
    kInvalidInput,
    kInnovationGate,
    kTimeAlignment,
    kNumericalFailure,
    kVrpnFault,
};

struct ObservationHealthConfig {
    std::size_t window_size{5};
    std::size_t fault_after_rejects{3};
    std::size_t recovery_after_accepts{3};
    double window_chi_square_gate{80.0};
};

class ObservationHealthTracker {
  public:
    static constexpr std::size_t kMaxWindowSize = 32u;

    void reset() {
        state_ = VrpnObservationState::kTrusted;
        consecutive_rejects_ = 0;
        consecutive_accepts_ = 0;
        chi_square_window_head_ = 0u;
        chi_square_window_size_ = 0u;
        chi_square_window_.fill(0.0);
        chi_square_window_sum_ = 0.0;
    }

    void setConfig(ObservationHealthConfig config) {
        config_ = config;
        if (config_.window_size == 0u) {
            config_.window_size = ObservationHealthConfig{}.window_size;
        }
        if (config_.window_size > kMaxWindowSize) {
            config_.window_size = kMaxWindowSize;
        }
        if (config_.fault_after_rejects == 0u) {
            config_.fault_after_rejects = ObservationHealthConfig{}.fault_after_rejects;
        }
        if (config_.recovery_after_accepts == 0u) {
            config_.recovery_after_accepts = ObservationHealthConfig{}.recovery_after_accepts;
        }
        if (!std::isfinite(config_.window_chi_square_gate) || config_.window_chi_square_gate <= 0.0) {
            config_.window_chi_square_gate = ObservationHealthConfig{}.window_chi_square_gate;
        }
        trimWindow();
    }

    void recordAccepted(double chi_square) {
        if (!std::isfinite(chi_square)) {
            recordRejected();
            return;
        }
        consecutive_rejects_ = 0;
        ++consecutive_accepts_;
        pushChiSquare(chi_square);
        if (state_ == VrpnObservationState::kFault) {
            state_ = VrpnObservationState::kRecovery;
        }
        if (state_ == VrpnObservationState::kRecovery &&
            consecutive_accepts_ >= config_.recovery_after_accepts) {
            state_ = VrpnObservationState::kTrusted;
        }
        if (state_ == VrpnObservationState::kSuspected &&
            consecutive_accepts_ >= config_.recovery_after_accepts) {
            state_ = VrpnObservationState::kTrusted;
        }
        if (chi_square_window_sum_ > config_.window_chi_square_gate) {
            state_ = VrpnObservationState::kSuspected;
        }
    }

    void recordRejected() {
        consecutive_accepts_ = 0;
        ++consecutive_rejects_;
        if (consecutive_rejects_ >= config_.fault_after_rejects) {
            state_ = VrpnObservationState::kFault;
        } else {
            state_ = VrpnObservationState::kSuspected;
        }
    }

    void recordRecoveryCandidate() {
        consecutive_rejects_ = 0;
        ++consecutive_accepts_;
        if (state_ == VrpnObservationState::kFault) {
            state_ = VrpnObservationState::kRecovery;
            return;
        }
        if ((state_ == VrpnObservationState::kRecovery || state_ == VrpnObservationState::kSuspected) &&
            consecutive_accepts_ >= config_.recovery_after_accepts) {
            state_ = VrpnObservationState::kTrusted;
        }
    }

    VrpnObservationState state() const { return state_; }

    double chiSquareWindowSum() const { return chi_square_window_sum_; }

    std::size_t consecutiveRejects() const { return consecutive_rejects_; }

    std::size_t consecutiveAccepts() const { return consecutive_accepts_; }

  private:
    void pushChiSquare(double chi_square) {
        if (!std::isfinite(chi_square)) {
            return;
        }
        if (chi_square < 0.0) {
            chi_square = 0.0;
        }
        if (chi_square_window_size_ < config_.window_size) {
            const std::size_t index = (chi_square_window_head_ + chi_square_window_size_) % kMaxWindowSize;
            chi_square_window_[index] = chi_square;
            ++chi_square_window_size_;
            chi_square_window_sum_ += chi_square;
            return;
        }
        chi_square_window_sum_ -= chi_square_window_[chi_square_window_head_];
        chi_square_window_[chi_square_window_head_] = chi_square;
        chi_square_window_sum_ += chi_square;
        chi_square_window_head_ = (chi_square_window_head_ + 1u) % kMaxWindowSize;
    }

    void trimWindow() {
        while (chi_square_window_size_ > config_.window_size) {
            chi_square_window_sum_ -= chi_square_window_[chi_square_window_head_];
            chi_square_window_[chi_square_window_head_] = 0.0;
            chi_square_window_head_ = (chi_square_window_head_ + 1u) % kMaxWindowSize;
            --chi_square_window_size_;
        }
        if (chi_square_window_sum_ < 0.0) {
            chi_square_window_sum_ = 0.0;
        }
    }

    ObservationHealthConfig config_{};
    VrpnObservationState state_{VrpnObservationState::kTrusted};
    std::size_t consecutive_rejects_{0};
    std::size_t consecutive_accepts_{0};
    std::array<double, kMaxWindowSize> chi_square_window_{};
    std::size_t chi_square_window_head_{0};
    std::size_t chi_square_window_size_{0};
    double chi_square_window_sum_{0.0};
};

} // namespace xgc2_math
