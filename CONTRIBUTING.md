# Contributing

Thanks for your interest in AlignGrasp. This is a competition-grade ROS 2
prototype, so changes that keep the algorithm core ROS-independent and the
offline tests green are welcome.

## Development Setup

The algorithm core (`core.py`, `temporal_recognition.py`, `pose_alignment.py`)
does not import ROS. To work on it, only Python and pytest are needed:

```bash
cd src/pick_action
python -m pytest -q
```

For the full workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Before Opening A Pull Request

```bash
ruff check .
ruff format --check .
python -m compileall -q src/pick_action/pick_action
cd src/pick_action && python -m pytest -q
```

## Guidelines

- Keep new algorithm logic in the ROS-independent modules and test it offline;
  keep ROS nodes as thin wrappers around it.
- Do not hardcode machine-specific absolute paths; use ROS parameters or the
  package `config/` directory.
- Add or update tests for every behavior change.
- Update `CHANGELOG.md` under `[Unreleased]` for user-visible changes.
