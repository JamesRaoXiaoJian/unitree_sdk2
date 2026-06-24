# MuJoCo Simulation Bridge (headless)
# Links: unitree_sdk2 (DDS) + MuJoCo C library (physics)

# Auto-detect MuJoCo from pip-installed package
# Try conda python first, then system python
execute_process(
    COMMAND bash -c "for p in /home/james/miniconda3/bin/python3 /home/james/miniconda3/bin/python python3 python; do if command -v $p >/dev/null 2>&1 && $p -c 'import mujoco' 2>/dev/null; then $p -c 'import mujoco, os; print(os.path.dirname(mujoco.__file__))'; exit 0; fi; done; exit 1"
    OUTPUT_VARIABLE MUJOCO_PY_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE MUJOCO_DETECT_RESULT
)

if(MUJOCO_PY_DIR AND EXISTS "${MUJOCO_PY_DIR}/include/mujoco/mujoco.h")
    set(MUJOCO_INCLUDE_DIR "${MUJOCO_PY_DIR}/include")
    file(GLOB MUJOCO_LIBS "${MUJOCO_PY_DIR}/libmujoco*.so*")
    if(MUJOCO_LIBS)
        list(GET MUJOCO_LIBS 0 MUJOCO_LIBRARY)
    endif()
endif()

# Fallback: search system paths
if(NOT MUJOCO_INCLUDE_DIR)
    find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
        PATHS /usr/local/include /usr/include)
endif()
if(NOT MUJOCO_LIBRARY)
    find_library(MUJOCO_LIBRARY NAMES mujoco
        PATHS /usr/local/lib /usr/lib)
endif()

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
    message(STATUS "MuJoCo: ${MUJOCO_LIBRARY}")

    add_executable(mujoco_sim_bridge mujoco_sim_bridge.cpp)
    target_include_directories(mujoco_sim_bridge PRIVATE ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(mujoco_sim_bridge
        unitree_sdk2
        ${MUJOCO_LIBRARY}
        Threads::Threads
        dl
    )
    set_target_properties(mujoco_sim_bridge PROPERTIES
        BUILD_RPATH "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    )
else()
    message(WARNING "MuJoCo not found. Install: pip install mujoco")
endif()
