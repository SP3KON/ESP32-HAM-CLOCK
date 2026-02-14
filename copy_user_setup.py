Import("env")
import shutil
import os

# Copy User_Setup.h to TFT_eSPI library directory before build
def copy_user_setup(*args, **kwargs):
    print("Copying User_Setup.h to TFT_eSPI library...")
    
    # Source file
    project_dir = env.get("PROJECT_DIR")
    source_file = os.path.join(project_dir, "User_Setup.h")
    
    # Find TFT_eSPI library directory
    lib_deps_dir = os.path.join(project_dir, ".pio", "libdeps", env.get("PIOENV"))
    tft_espi_dir = os.path.join(lib_deps_dir, "TFT_eSPI")
    
    if os.path.exists(source_file) and os.path.exists(tft_espi_dir):
        dest_file = os.path.join(tft_espi_dir, "User_Setup.h")
        shutil.copy2(source_file, dest_file)
        print(f"Copied {source_file} to {dest_file}")
    else:
        print(f"Warning: Source file or TFT_eSPI directory not found")
        if not os.path.exists(source_file):
            print(f"  Source file not found: {source_file}")
        if not os.path.exists(tft_espi_dir):
            print(f"  TFT_eSPI directory not found: {tft_espi_dir}")

env.AddPreAction("buildprog", copy_user_setup)
