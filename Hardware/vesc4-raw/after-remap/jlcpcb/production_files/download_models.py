import os
import subprocess
import pandas as pd

# 1. Load the BOM file
bom_file = 'BOM-after-remap.csv'
df = pd.read_csv(bom_file)

# 2. Extract all unique, non-null LCSC IDs
lcsc_ids = df['LCSC'].dropna().unique()
print(f"Found {len(lcsc_ids)} unique components to process.")

# 3. Create a dedicated output folder and get its ABSOLUTE path
output_dir = os.path.abspath("./easyeda_libs")
os.makedirs(output_dir, exist_ok=True)

# Define the absolute prefix for the library files
output_prefix = os.path.join(output_dir, "easyeda_parts")

# 4. Loop through and fetch each asset automatically
for lcsc_id in lcsc_ids:
    print(f"\n--> Fetching {lcsc_id} (Footprint, Symbol & 3D Model)...")
    
    # ADDED: "--overwrite" to force past duplicate footprint errors
    command = [
        "python", "-m", "easyeda2kicad",
        "--full",
        f"--lcsc_id={lcsc_id}",
        "--output", output_prefix,
        "--project-relative",
        "--overwrite"
    ]
    
    try:
        # Executes the terminal command directly from Python
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"⚠️ Failed to download {lcsc_id}. Skipping...")
    except FileNotFoundError:
        print("❌ Error: Python interpreter couldn't be reached.")
        break

print("\n🎉 Success! Process complete. Check your './easyeda_libs' directory.")