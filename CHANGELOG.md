# Changelog

All notable changes for each released versions (since v0.3.1) will be documented
here.

## v0.5.1 (2024-05-24)

This is a hot fix release mainly to trigger the release workflow so that we have
all the precompiled binary packages correctly uploaded. There is no change in
the package.

**Please refer to the change log for v0.5.0 for actually changes since v0.4.2.**

## v0.5.0 (2024-05-24)

**This is a major release that changes many internal things. Please report any
breakage you find. Thanks!**

This is the first release that supports Windows, also the first release without
the seemingly unmaintained SNAP library.

### Added

- Windows support and compiled binary releases for Windows ([GH-55]).
- Compiled binary releases against musl ([GH-55]).
- Compiled fat binary releases for macOS (mainly to support older machines with
  Intel chips).
- Time-dependent and OD-dependent adaptive ratio for vehicle routing ([GH-58]).

### Changed

- Update pybind11 to v2.12.0.

### Removed

- Dependency on the SNAP library ([GH-50], [GH-55]).

[GH-50]: https://github.com/maccmu/macposts/pull/50
[GH-55]: https://github.com/maccmu/macposts/pull/55
[GH-58]: https://github.com/maccmu/macposts/pull/58

## v0.4.2 (2024-03-18)

Routine maintenance and add a new graph module to prepare for droping SNAP.

### Added

- Enable infering version from Git tags ([GH-45]).
- Add a new graph module ([GH-46]).
- Add more options to output vehicle trajectories ([GH-48]).

[GH-45]: https://github.com/maccmu/macposts/pull/45
[GH-46]: https://github.com/maccmu/macposts/pull/46
[GH-48]: https://github.com/maccmu/macposts/pull/48

## v0.4.1 (2023-11-07)

This is a hot fix release to address two issues with the Python package.

### Fixed

- Update version number for the Python package ([GH-41]).

[GH-41]: https://github.com/maccmu/macposts/pull/41

## v0.4.0 (2023-11-07)

**This release has some issues (see [GH-39]), please use v0.4.1 instead.**

[GH-39]: https://github.com/maccmu/macposts/issues/39

### Added

- Add module for modeling electric vehicles ([GH-32]).
- Add support for modeling incidents ([GH-33]).
- Add support for Python 3.12 ([GH-31]).

[GH-31]: https://github.com/maccmu/macposts/pull/31
[GH-32]: https://github.com/maccmu/macposts/pull/32
[GH-33]: https://github.com/maccmu/macposts/pull/33

### Changed

- Explicitly require CMake≥3.10 for building the project (df2a8da).

### Fixed

- Correct DSO marginal cost of CTM links ([GH-34]).

[GH-34]: https://github.com/maccmu/macposts/pull/34

### Removed

- Drop support for Python 3.7, which has reached its EOL (6f3af1c).

## v0.3.1 (2023-06-16)

### Added

- Add module for delivery traffic modeling.
- New DTA example for Sioux Falls network ([GH-23]).

[GH-23]: https://github.com/maccmu/macposts/pull/23

### Fixed

- Correct link marginal cost calculation for DSO.
- Correct gradient projection for DUE.
