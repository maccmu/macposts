# macposts

A toolkit for transportation network modeling. To install from the repository,
first ensure a working C++ toolchain (note that MSVC is not supported now and we
appreciate any help on the Windows build) and CMake ≥ 3.10. Then clone this
repository, initialize and clone all submodules, and run at the project root:

```sh
pip install .
```

For development, run

```sh
pip install -e .[dev]
```

instead to also install the development tools and enable editable mode. If you
need debug information for the C++ library (and also enable other settings for
debugging), set the environment variable `DEBUG` to `1` before installation.

For normal users, you can instead use the precompiled wheels of tagged releases
for certain platforms and Python versions on the GitHub [releases] page. There
are “sdist” packages on the [releases] page as well. To install an sdist
package, a C++ toolchain and CMake are required.

[releases]: https://github.com/kunhtkun/macposts/releases

## Usage

Currently there is nothing that could be considered as the documentation. If you
still want to use it, you may check the ‘examples’ directory in this repository.

## Frequently asked questions

* How is this project related to [MAC-POSTS]?

  This project is the maintained fork of MAC-POSTS. We took over the maintenance
  work a few years ago. At that time, we found that the Git repository was very
  large due to the data files in tree, and the Git commit history was not very
  formally maintained. So we decided to start clean and made this new
  repository.

* Will this be compatible with MAC-POSTS?

  Mostly yes. The Python API should be compatible (except that the library is
  now named `macpsots` instead of `MNMAPI`) while breaking changes may have been
  introduced to the underlying C++ library (we are unsure because we do not know
  which functions are internal). We will maintain the backward compatibility for
  the Python binding in the future. So it is recommended to only use the Python
  library and treat the whole C++ library as internal.

* Where is the Git repository?

  The Git repository is hosted on an internal server currently. However, we also
  have a GitHub mirror for the repository, for which the main branch is
  regularly updated (other branches may not be available).

  This is because currently we still consider this project in the pre-alpha
  stage and want to craft it a bit more before we can comfortably make it really
  public.

* How to report issues and send patches?

  As said, we think this project is still in its infancy and are developing it
  mainly internally. So the issues page on GitHub is closed. If you want to
  report issues, please feel free to contact any of the committers in the Git
  history. The pull requests page is still open and we will review and accept
  patches there. You can also send patches via email as well.

[MAC-POSTS]: https://github.com/Lemma1/MAC-POSTS

## Contributors

- Wei Ma: author and former maintainer
- Xidong Pi: co-author and former maintainer
- Pengji Zhang: current maintainer
- Pinchao Zhang: contributor
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
