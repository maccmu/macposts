import re
from pathlib import Path

SEED = 0

here = Path(__file__).resolve().parent
fixtures_directories = [here / "fixtures"]
pytest_plugins = []


def list_files(directory, exclude=None):
    if not (exclude is None or isinstance(exclude, re.Pattern)):
        exclude = re.compile(exclude)
    for path in directory.iterdir():
        if exclude is None or not exclude.match(path.name):
            if path.is_dir():
                yield from list_files(path, exclude)
            else:
                yield path.resolve()


def as_plugin(path):
    root = here.parent
    parts = path.relative_to(root).parts
    name = ".".join(parts).rstrip(".py")
    return name


# Add all fixtures
for directory in fixtures_directories:
    files = list_files(directory, r"^_")
    pytest_plugins += [
        as_plugin(path) for path in files if path.suffix == ".py"
    ]
