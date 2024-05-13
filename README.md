# MAC-POSTS

MAC-POSTS, the abbreviation of *Mobility data Analytics Center - Prediction,
Optimization, and Simulation toolkit for Transportation Systems* is a toolkit
for dynamic transportation network modeling developed by [Mobility data
Analytics Center (MAC)][mac] at Carnegie Mellon University.

## Installation

MAC-POSTS works as a Python library. It currently supports Linux, Windows, and
macOS platforms.

To install from the repository, first ensure a working C++ toolchain (`GCC` or 
`Clang` on Linux and macOS and `MSVC` on Windows), `CMake >= 3.10`, and `Git` 
are installed. 

Then open terminal or command prompt, run the following command to clone the 
repository, initialize and clone all submodules.

```sh
git clone --recurse-submodules https://github.com/maccmu/macposts.git
```

Create a `Python>=3.8` virtual environment and run at the project root (i.e., 
`path_to_store_cloned_macposts_repo/macposts/`):

```sh
pip install .
```

This will install the `main` branch by default.

For development, run

```sh
pip install -e .[dev]
```

instead to install the `dev` branch and enable editable mode. Or you can manually
 check out `dev` branch first

```sh
git checkout dev
pip install -e .
```

If you need debug information for the C++ library (and also enable other settings 
for debugging), set the environment variable `DEBUG` to `1` in the current shell 
session before installation.

On Linux and macOS, just type the following in the terminal

```sh
DEBUG=1
```

On Windows, type the following instead in the command prompt

```sh
set DEBUG=1
```

For normal users, you can instead use the precompiled wheels of tagged releases
for certain platforms and Python versions on the GitHub [releases] page. There
are “sdist” packages on the [releases] page as well. To install an sdist
package, a C++ toolchain are required.

## Usage

Please refer to this [link][documentation] for the documentation. You may also
check the ‘examples’ directory for some working examples in this repository.

**CAVEAT:** *Do not run macposts on untrusted inputs. Currently it uses a rather
crude data file reader/parser and may have some security vulnerabilities,
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

* How to report issues and send patches?

  If you want to report issues, please feel free to contact us via
  [macenter@andrew.cmu.edu][macenter]. The pull requests page is open and we
  will review and accept patches there. You can also send patches via email as
  well.

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
- Eigen: lib/eigen/COPYING.* (multiple licenses)

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
