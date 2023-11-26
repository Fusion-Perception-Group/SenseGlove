from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

env.Append(
    LINKFLAGS=[
        #"-specs=nosys.specs",
        "-fexceptions",
        "-ffunction-sections",        
        "-fdata-sections",
        "-mcpu=cortex-m4",
        "-march=armv7e-m",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
    ],
)