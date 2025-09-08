#include <type_traits>

#include "wuzy/wuzy.h"

namespace wuzy {
wuzy_allocator* default_allocator();

template <typename T>
T* allocate(wuzy_allocator* alloc, size_t count = 1)
{
    alloc = alloc ? alloc : default_allocator();
    // NOLINTNEXTLINE(bugprone-sizeof-expression)
    auto ptr = reinterpret_cast<T*>(alloc->allocate(sizeof(T) * count, alloc->ctx));
    for (size_t i = 0; i < count; ++i) {
        new (ptr + i) T {};
    }
    return ptr;
}

template <typename T>
T* reallocate(wuzy_allocator* alloc, T* ptr, size_t old_count, size_t new_count)
{
    static_assert(std::is_trivially_copyable_v<T>);
    alloc = alloc ? alloc : default_allocator();
    ptr = reinterpret_cast<T*>(
        alloc->reallocate(ptr, sizeof(T) * old_count, sizeof(T) * new_count, alloc->ctx));
    return ptr;
}

template <typename T>
void deallocate(wuzy_allocator* alloc, T* ptr, size_t count = 1)
{
    alloc = alloc ? alloc : default_allocator();
    for (size_t i = 0; i < count; ++i) {
        (ptr + i)->~T();
    }
    // NOLINTNEXTLINE(bugprone-sizeof-expression)
    return alloc->deallocate(ptr, sizeof(T) * count, alloc->ctx);
}
}