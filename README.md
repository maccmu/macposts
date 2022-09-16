# macposts

A toolkit for transportation network modeling. To install from the repository,
first ensure a working C++ toolchain (note that MSVC is not supported now and we
appreciate any help on the Windows build) and CMake ≥ 3.10. Then clone this
repository (with `--recurse-submodules` ), and run at the project root:

```sh
pip install .
```

For development, run

```sh
pip install -e .[dev]
```

instead to also install the development tools and enable editable mode.

## Contributors

- Wei Ma: author and former maintainer
- Xidong Pi: co-author and former maintainer
- Pengji Zhang: current maintainer
- Sean Qian: advisor

## License

macposts is licensed under MIT license. See also the LICENSE file.

macposts depends on three external libraries under the lib directory. You may
want to check their licenses as well:

- pybind11: lib/pybinb11/LICENSE
- Eigen: lib/eigen/COPYING.* (multiple licenses)
- Snap: lib/snap/License.txt

## Acknowledgment

This project is funded in part by Traffic 21 Institute, Carnegie Mellon
University's Mobility21, Technologies for Safe and Efficient Transportation
(T-SET), US Department of Transportation (DOT), US Department of Energy (DOE).
The contents of this project reflect the views of the authors, who are
responsible for the facts and the accuracy of the information presented herein.
The US Government assumes no liability for the contents or use thereof.
