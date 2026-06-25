#ifndef XGC2_MATH_STATUS_HPP
#define XGC2_MATH_STATUS_HPP

namespace xgc2_math {

template <typename Status> struct StatusDescriptor {
    Status status;
    const char* name;
};

template <typename Status> struct StatusRegistry;

template <typename Status> inline const auto& registeredStatuses() { return StatusRegistry<Status>::statuses; }

template <typename Status> inline const StatusDescriptor<Status>* statusDescriptor(Status status) {
    for (const auto& descriptor : registeredStatuses<Status>()) {
        if (descriptor.status == status) {
            return &descriptor;
        }
    }
    return nullptr;
}

template <typename Status> inline const char* toString(Status status) {
    const auto* descriptor = statusDescriptor(status);
    return descriptor != nullptr ? descriptor->name : "unknown";
}

} // namespace xgc2_math

#endif // XGC2_MATH_STATUS_HPP
