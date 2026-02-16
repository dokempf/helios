# Installing and Testing

- All testings needs to be done in a Conda environment called `helios-dev`. If it is not activated, stop and ask me to activate it.
- If the build directory `build` does not exist, you can create it with `python -m pip install --no-build-isolation --config-settings=build-dir="build" -v -e .`
- If the build directory already exists, you need to recompile using `ninja` in the build directory.
- Always perform C++ tests first by running `ctest` in the build directory
- Then perform Python testing by running `pytest -vv` in the project root.
- Ensure that your changes pass `pre-commit run -a`. If it fails, run it a second time, because some hooks might have reformatted the source code and pass on the second attempt. Fix any issues that persist after the second attempt.
- Ensure that your changes are covered by tests by running `pytest --cov` with arguments suited for your automatic inspection.
