import os
import subprocess

def replace_header_paths(root_dir):
    for subdir, _, files in os.walk(root_dir):
        for file in files:
            filepath = os.path.join(subdir, file)
            if not os.path.islink(filepath):
                # subprocess.run(['sed', '-i', f's\/#include "{file}"\/#include "{os.path.relpath(subdir, root_dir).replace('/', "\/")}\\/{file}"\/g', ])
                subprocess.run(f"find {root_dir} -type f -exec sed -i 's/\"{file}\"/\"{os.path.relpath(subdir, root_dir).replace('/', r"\/")}\\/{file}\"/g' {"{}"} +", shell=True)
                print(f'Replaced header paths in {filepath}')

# Replace header paths in all files under ./lib/ELFE
root_directory = './lib/ELFE'
replace_header_paths(root_directory)
