// register_types.cpp — GDExtension entry point.
//
// Design authority: docs/architecture/godot_plugin.md §register_types Entry Point
//
// Registers SimulationReceiver with Godot's ClassDB at
// MODULE_INITIALIZATION_LEVEL_SCENE.  On Windows, WSAStartup()/WSACleanup() are
// called here so the socket is ready before any SimulationReceiver node runs
// _ready().

#include "register_types.hpp"
#include "SimulationReceiver.hpp"

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#ifdef _WIN32
#include <winsock2.h>
#endif

using namespace godot;

void initialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
        return;

#ifdef _WIN32
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif

    ClassDB::register_class<SimulationReceiver>();
}

void uninitialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
        return;

#ifdef _WIN32
    WSACleanup();
#endif
}

extern "C" GDExtensionBool GDE_EXPORT liteaero_sim_init(
    GDExtensionInterfaceGetProcAddress p_get_proc_address,
    const GDExtensionClassLibraryPtr   p_library,
    GDExtensionInitialization         *r_initialization)
{
    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_liteaero_sim);
    init_obj.register_terminator(uninitialize_liteaero_sim);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
