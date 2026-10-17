#include <algorithm>
#include <dlfcn.h>
#include <string>

#include <gtest/gtest.h>

#include "finenav_core/map/detail/map_registry.hpp"

TEST(PluginRegistration, SharedLibraryRegistersTemperalVoxelMapByName) {
    void* handle = dlopen(TEMPERAL_VOXEL_MAP_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    ASSERT_NE(handle, nullptr) << dlerror();

    const auto names = finenav::core::detail::MapRegistry::instance().mapNames();
    EXPECT_NE(std::find(names.begin(), names.end(), "Temperal_voxel_map"), names.end());

    // Keep the DSO loaded until process exit: the registry stores factories
    // whose code belongs to this library.
}
