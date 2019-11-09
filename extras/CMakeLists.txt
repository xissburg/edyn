if (UNIX)
    set(EDyn_EXTRAS_SOURCES
        src/unix/time.cpp
    )
endif()

include_directories($<TARGET_PROPERTY:EDyn,INCLUDE_DIRECTORIES>)
add_compile_options($<TARGET_PROPERTY:EDyn,COMPILE_OPTIONS>)

add_library(EDyn_extras STATIC ${EDyn_EXTRAS_SOURCES})

target_include_directories(EDyn_extras 
    PUBLIC ${EDyn_SOURCE_DIR}/extras/include
)