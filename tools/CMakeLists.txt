# ---------------------------------------------------------------------------
# LiteAeroSim standalone tools
#
# Each tool is a separate executable that links liteaerosim.
# Tools are not part of the library and are excluded from the lib glob.
# ---------------------------------------------------------------------------

# joystick_verify: streams ManualInputFrame to stdout for notebook display.
# Requires SDL2 (transitively via liteaerosim).
add_executable(joystick_verify
    ${CMAKE_CURRENT_SOURCE_DIR}/joystick_verify.cpp
)
target_link_libraries(joystick_verify
    PRIVATE
        liteaerosim
        liteaerosim_proto
        SDL2::SDL2
)
target_compile_features(joystick_verify PRIVATE cxx_std_17)

# Force console subsystem so joystick_verify can write to stdout.
if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(joystick_verify PRIVATE -mconsole)
endif()

# Copy SDL2.dll next to joystick_verify.exe at build time so the executable is
# self-contained and does not require ucrt64/bin on PATH at runtime.
# SDL2::SDL2 is a Conan IMPORTED target; its DLL location is in
# IMPORTED_LOCATION or IMPORTED_LOCATION_RELEASE rather than TARGET_FILE.
if(WIN32)
    get_target_property(_sdl2_dll SDL2::SDL2 IMPORTED_LOCATION_RELEASE)
    if(NOT _sdl2_dll)
        get_target_property(_sdl2_dll SDL2::SDL2 IMPORTED_LOCATION)
    endif()
    if(_sdl2_dll AND EXISTS "${_sdl2_dll}")
        add_custom_command(TARGET joystick_verify POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_sdl2_dll}"
                $<TARGET_FILE_DIR:joystick_verify>
            COMMENT "Deploying SDL2.dll alongside joystick_verify"
        )
    else()
        message(WARNING
            "SDL2.dll not found via IMPORTED_LOCATION on SDL2::SDL2 — "
            "joystick_verify may require ucrt64/bin on PATH at runtime."
        )
    endif()
endif()

# ---------------------------------------------------------------------------
# live_sim: real-time simulation launcher with UDP broadcast to Godot 4.
# Design authority: docs/architecture/live_sim_view.md
# ---------------------------------------------------------------------------
add_executable(live_sim
    ${CMAKE_CURRENT_SOURCE_DIR}/live_sim.cpp
)
target_link_libraries(live_sim
    PRIVATE
        liteaerosim
        liteaerosim_proto
        SDL2::SDL2
        $<$<PLATFORM_ID:Windows>:ws2_32>
)
target_compile_features(live_sim PRIVATE cxx_std_17)

# Force console subsystem so live_sim can write to stdout.
if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(live_sim PRIVATE -mconsole)
endif()

# Deploy SDL2.dll alongside live_sim.exe on Windows.
if(WIN32 AND _sdl2_dll AND EXISTS "${_sdl2_dll}")
    add_custom_command(TARGET live_sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_sdl2_dll}"
            $<TARGET_FILE_DIR:live_sim>
        COMMENT "Deploying SDL2.dll alongside live_sim"
    )
endif()
