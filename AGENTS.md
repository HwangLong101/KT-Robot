# Repository Guidelines

## Project Structure

- `pure_pursuit_baseline/` reproduces the upstream PythonRobotics example. The pinned source and license live in `vendor/`, with provenance in `source_manifest.json`.
- `pure_pursuit_diffdrive/` contains the original differential-drive implementation. `pure_pursuit_evaluation/` compares both speed policies across 42 nominal configurations. Each has its own Python dependencies and tests.
- `simulink/` contains the nominal and robustness models, editable MATLAB Function sources, build and evaluation scripts, and generated results. Root-level Word reports and `README_ROBOT_CHO_NHOM.md` document the project.

## Build and Run

Run Python commands from the relevant project directory. Install its dependencies with `python -m pip install -r requirements.txt`. In `pure_pursuit_evaluation/`, run `python run_robot.py --method curvature --scenario corners --animate` for an interactive example or `python run_evaluation.py` to regenerate the nominal results.

From `simulink/`, use MATLAB and Simulink R2025b:

- `matlab -batch "build_kt_robot; build_kt_robustness"` rebuilds both models.
- `matlab -batch "evaluate_kt_robot"` compares the nominal model with the Python CSV fixtures.
- `matlab -batch "evaluate_kt_robustness('outputs_robustness',false)"` runs the robustness study and skips the full trace archive.

## Coding and Testing

Use four-space indentation, Python `snake_case` for modules and functions, and MATLAB function names that match their `.m` filenames. Follow the existing `pp_` and `kt_robust_` prefixes. Use SI units and retain unit suffixes in exported column names. Edit MATLAB Function sources and rebuild the corresponding `.slx` files. The repository has no configured formatter or linter.

Run `python -m unittest -v` separately in `pure_pursuit_diffdrive/` and `pure_pursuit_evaluation/`. Add Python regressions to `test_verification.py` using `test_*` methods. From `simulink/`, run `matlab -batch "run_kt_checks; check_kt_robustness"` for model checks. Preserve scenario parameters and seeds when comparing results.

## Commits and Pull Requests

Git history starts with a single `Initial commit`. Use short imperative subjects for new commits. Explain what changed, list the checks run, and link a related issue when available. Include updated plots or diagrams when those outputs change. Keep Simulink caches out of commits and document intentional changes to reference data.
