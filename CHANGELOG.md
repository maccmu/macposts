# Changelog

All notable changes for each released versions (since v0.3.1) will be documented
here.

## v0.4.1 (2023-11-07)

This is a hot fix release to address two issues with the Python package.

### Fixed

- Update version number for the Python package ([GH-41]).
- Fix release workflow on GitHub ([GH-40]).

[GH-40]: https://github.com/maccmu/macposts/pull/40
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
