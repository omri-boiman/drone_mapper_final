#pragma once

#include <dlfcn.h>

#include <utility>

namespace simulator {

// RAII wrapper around a dlopen() handle. Move-only; dlclose()s on destruction.
class DynamicLibraryHandle {
public:
    DynamicLibraryHandle() = default;
    explicit DynamicLibraryHandle(void* handle) noexcept : handle_(handle) {}

    DynamicLibraryHandle(const DynamicLibraryHandle&) = delete;
    DynamicLibraryHandle& operator=(const DynamicLibraryHandle&) = delete;

    DynamicLibraryHandle(DynamicLibraryHandle&& other) noexcept
        : handle_(std::exchange(other.handle_, nullptr)) {}

    DynamicLibraryHandle& operator=(DynamicLibraryHandle&& other) noexcept {
        if (this != &other) {
            reset();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    ~DynamicLibraryHandle() { reset(); }

    void reset() {
        if (handle_) {
            ::dlclose(handle_);
            handle_ = nullptr;
        }
    }

    [[nodiscard]] bool valid() const noexcept { return handle_ != nullptr; }

private:
    void* handle_ = nullptr;
};

} // namespace simulator
