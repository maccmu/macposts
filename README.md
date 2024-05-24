# MAC-POSTS

MAC-POSTS, the abbreviation of *Mobility data Analytics Center – Prediction,
Optimization, and Simulation toolkit for Transportation Systems* is a toolkit
for dynamic transportation network modeling.

Developed by [Mobility data Analytics Center (MAC)][mac] at Carnegie Mellon
University, this package implements many classic dynamic transportation network
models and also new models proposed by MAC members. Besides, it has served as
one building block for many other models and research projects (example
[1][eg1], [2][eg2], and [3][eg3]).

As such, this package used to be treated as an internal research project of MAC
lab, and admittedly the code base is messy and the interface is hard to use.
However, we are working hard to make it a generally usable and useful toolkit
for dynamic transportation network modeling. We would really appreciate it if
you try to use it and give us some feedback, comments, suggestions, or
criticisms.

## Installation

MAC-POSTS works as a Python library. It currently supports 64-bit Linux,
Windows, and macOS platforms.

Normal users are advised to use the precompiled packages (or “wheels” in Python
jargon), which are on the [GitHub Releases page][releases]. You will see a large
variety of files on that page with some cryptic [compatibility tags]. If you do
not know which one to use, here are our recommended steps:

1. Check your Python version. If you are using Python 3.12, then you need to use
   those files including the string “cp312”.
2. Now from those files, select the one that works for your platform:
   - On Linux using glibc (e.g., Debian, Fedora, Ubuntu, etc.), search for
     “manylinux”. On Linux using musl libc (e.g., Alpine Linux), search for
     “musllinux”.
   - On Windows, search for “win”.
   - On macOS with the new silicon chips (the “M” chips), search for “arm64”. On
     macOS with the older Intel chips, search for “universal2”.

For example, if you are using Python 3.12 on Debian Bookworm, you can download
the `macposts-0.5.0-cp312-cp312-manylinux_2_17_x86_64.manylinux2014_x86_64.whl`,
and run in Bash (or other POSIX shells):

```sh
pip install macposts-0.5.0-cp312-cp312-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

to install MAC-POSTS v0.5.0. In the future we may publish this package to the
official PyPI registry to ease this process, but we would like to first improve
the quality of this package (see [GH-17]).

If you are using some platform or Python version for which there is no
precompiled package, you may try to install the package from source using the
“sdist” package (e.g., `macposts-0.5.0.tar.gz` for MAC-POSTS v0.5.0). To install
such a package, first ensure a working C++ toolchain. We recommend:

- GCC or Clang on Linux.
- Clang on macOS (via Xcode).
- MSVC on Windows (via Visual Studio).

Then run:

```sh
pip install macposts-0.5.0.tar.gz
```

which will compile and install MAC-POSTS.

MAC-POSTS is under active development and you may often see bug fixes or new
features missing in the latest tagged release. If you really need those, you
could install the package from the repository ([instructions](#development)).

## Usage

Please refer to this [link][documentation] for the documentation. You may also
check the ‘examples’ directory for some working examples in this repository.

**CAVEAT:** *Do not run MAC-POSTS on untrusted inputs. Currently it uses a
rather crude data file reader/parser and may have some security vulnerabilities,
including remote code execution (RCE).*

## Frequently asked questions

* How is this project related to the previous [MAC-POSTS]?

  This project is the maintained fork of MAC-POSTS. We took over the maintenance
  work a few years ago. At that time, we found that the Git repository was very
  large due to the data files in tree, and the Git commit history was not very
  formally maintained. So we decided to start clean and made this new
  repository. Moreover, we corrected many bugs and added more features.

* Will this be compatible with the previous MAC-POSTS?

  Mostly yes. The Python API should be compatible (except that the library is
  now named `macposts` instead of `MNMAPI`) while breaking changes may have been
  introduced to the underlying C++ library (we are unsure because we do not know
  which functions are internal). We will maintain the backward compatibility for
  the Python binding in the future. So it is recommended to only use the Python
  library and treat the whole C++ library as internal.

## Development

### Prerequisites

For the development of MAC-POSTS, you need to install:

- A C++ toolchain.
  + On Linux: GCC or Clang.
  + On macOS: Clang (Xcode).
  + On Windows: MSVC (Visual Studio).
- Git.
- CMake (≥3.10).

On Windows, it is also recommended to install a POSIX-compliant shell (e.g.,
Bash), although that is not required.

### Clone the repository

We use Git submodules to vendor third-party libraries. To clone the repository
and all submodules, run:

```sh
git clone --recurse-submodules https://github.com/maccmu/macposts.git
```

Then get into the project root and change to the `dev` branch (in a
POSIX-compliant shell or PowerShell):

```sh
cd macposts
git checkout dev
```

### Install the package

For development, we usually install the package using the following command (in
a POSIX-compliant shell):

```sh
DEBUG=1 pip install -e .[dev]
```

`DEBUG=1` means that this is a debug build and so the debug configuration will
be used. Most notably, debug information will not be stripped from the compiled
binary, and so we can use GDB with it. In PowerShell, you may run:

```powershell
$env:DEBUG = 1
pip install -e .[dev]
```

Note that, unlike the command for POSIX shells, this will set `DEBUG` for the
whole PowerShell session.

### Development workflow

The normal development workflow is:

- Edit the source files.
- If C++ files are changed, rebuild the package.
- Test.

To rebuild the package, we may reinstall the package. However, that is slow and
wasteful. Oftentimes we only need to recompile a small portion of files but
reinstalling the package will compile everything again from scratch. A better
way is to use CMake.

First, we need to configure the project:

```sh
cmake -DCMAKE_BUILD_TYPE=Debug -S . -B build
```

We only need to run this command once for a newly cloned repository. Then after
making changes to the source files, run:

```sh
cmake --build build
```

to (re)compile the project. This will produce a file ‘build/_macposts_ext.\*’
(the suffix indicated by ‘\*’ varies depending on your tools used). Finally,
copy that ‘build/_macposts_ext.\*’ file to the project root, where there should
be an existing file with the same name.

To run the tests, use:

```sh
pytest
```

We value testing but that was historically overlooked. For new features or bug
fixes, please consider adding some test cases as well.

## Contributors

### Maintainers

- Qiling Zou: maintainer;
- Pengji Zhang: co-maintainer.

### Previous maintainers

- Wei Ma: author and former maintainer;
- Xidong Pi: co-author and former maintainer.

### Contributors

- Pinchao Zhang: contributor;
- Sean Qian: advisor.

## License

MAC-POSTS is licensed under MIT license. See also the LICENSE file.

MAC-POSTS depends on two external libraries under the lib directory. You may
want to check their licenses as well:

- pybind11: lib/pybinb11/LICENSE
- Eigen: lib/eigen/COPYING.\* (multiple licenses)

## Acknowledgment

This project is funded in part by Traffic 21 Institute, Carnegie Mellon
University's Mobility21, Technologies for Safe and Efficient Transportation
(T-SET), US Department of Transportation (DOT), and US Department of Energy
(DOE). The contents of this project reflect the views of the authors, who are
responsible for the facts and the accuracy of the information presented herein.
The US Government assumes no liability for the contents or use thereof.

[mac]: https://mac.heinz.cmu.edu/
[releases]: https://github.com/maccmu/macposts/releases
[documentation]: https://github.com/maccmu/macposts-documentations/blob/main/MAC_POSTS_users_manual.pdf
[MAC-POSTS]: https://github.com/Lemma1/MAC-POSTS
[macenter]: mailto:macenter@andrew.cmu.edu
[eg1]: https://doi.org/10.1016/j.trc.2019.05.011
[eg2]: https://doi.org/10.1016/j.trc.2020.102747
[eg3]: https://trid.trb.org/View/1573278
[compatibility tags]: https://packaging.python.org/en/latest/specifications/platform-compatibility-tags/#platform-compatibility-tags
[GH-17]: https://github.com/maccmu/macposts/issues/17
