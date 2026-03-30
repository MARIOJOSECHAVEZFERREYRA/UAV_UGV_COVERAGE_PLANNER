import os
import sys
import glob


def pick_json(root_dir: str) -> str:
    data_dir = os.path.join(root_dir, 'data', 'test_fields')
    files = sorted(glob.glob(os.path.join(data_dir, '**', '*.json'), recursive=True))

    if not files:
        print(f'No JSON files found under {data_dir}')
        sys.exit(1)

    print('\nAvailable field files:')
    for i, f in enumerate(files):
        rel = os.path.relpath(f, root_dir)
        print(f'  [{i+1}] {rel}')

    choice = input('\nEnter number (or full path): ').strip()
    if choice.isdigit():
        return files[int(choice) - 1]
    return choice
