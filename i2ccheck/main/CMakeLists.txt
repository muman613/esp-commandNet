set(SOURCES main.c i2c.c i2c.h console.c console.h)
idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS ".")
