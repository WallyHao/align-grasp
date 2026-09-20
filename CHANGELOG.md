# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- MIT `LICENSE`, `CONTRIBUTING.md` and a GitHub Actions workflow running Ruff,
  a byte-compile check and the offline unit tests.
- An offline unit-test suite for the ROS-independent algorithm core
  (`core.py`, `temporal_recognition.py`, `pose_alignment.py`).
- A `ruff.toml` and a pytest configuration.
- An English `README.md`; the detailed Chinese mode reference moved to
  `docs/modes.zh.md` and the tool-control notes to `docs/ares_tool_control.md`.

### Fixed

- The action server imported `ToolAction` from `r2_interfaces`, which is not in
  this repository; it now imports `ares_tool_interfaces` with a fallback, and
  `package.xml` depends on the shipped package.

### Changed

- Renamed the project to **AlignGrasp** and unified the maintainer identity.
- Applied Ruff formatting and safe lint fixes across the Python sources.

### Removed

- `AGENTS.md` (internal agent notes).
