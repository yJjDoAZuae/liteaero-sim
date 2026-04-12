#pragma once
// register_types.hpp — GDExtension entry point declarations.
//
// Design authority: docs/architecture/godot_plugin.md §register_types Entry Point

#include <godot_cpp/core/class_db.hpp>

void initialize_liteaero_sim(godot::ModuleInitializationLevel p_level);
void uninitialize_liteaero_sim(godot::ModuleInitializationLevel p_level);
