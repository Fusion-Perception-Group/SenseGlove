from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

env.Append(
    LINKFLAGS=[
        "-fdata-sections",
        "-mcpu=cortex-m4",
        "-march=armv7e-m",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
    ]
)