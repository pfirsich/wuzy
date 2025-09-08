#include "shared.hpp"

#include <cstdlib>

namespace wuzy {
wuzy_allocator* default_allocator()
{
    static wuzy_allocator alloc {
        .allocate = [](size_t size, void*) -> void* { return std::malloc(size); },
        .reallocate = [](void* ptr, size_t, size_t new_size, void*) -> void* {
            return std::realloc(ptr, new_size);
        },
        .deallocate = [](void* ptr, size_t, void*) { std::free(ptr); },
        .ctx = nullptr,
    };
    return &alloc;
}
}