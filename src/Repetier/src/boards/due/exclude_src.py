Import("env")

def skip_file(node):
    # to ignore file from a build process, just return None
    return None

env.AddBuildMiddleware(skip_file, "*/arduino/USB/*.cpp")
env.AddBuildMiddleware(skip_file, "*/arduino/main.cpp")
