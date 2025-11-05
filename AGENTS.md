# Repository Guidelines

## Project Structure & Module Organization
- `xcore/` core C++ library and interfaces.
- `modules/` acceleration backends: `ocl/`, `gles/`, `vulkan/`, `soft/`, `ocv/`, `render/`, `dnn/`, `isp/`.
- `plugins/` 3A and smart analysis plugins.
- `wrapper/gstreamer/` GStreamer elements (e.g., xcamsrc/xcamfilter).
- `shaders/` kernels: `cl/`, `glsl/`, `spv/`.
- `capi/` C API façade; `tests/` test programs and assets; `tools/` helper scripts; `doc/` Doxygen; `pkgconfig/`, `m4/` build files.

## Build, Test, and Development Commands
- Configure: `./autogen.sh --prefix=/usr --enable-gst --enable-libcl [--enable-opencv --enable-gles --enable-vulkan --enable-dnn --enable-debug]`
- Build: `make -j$(nproc)`
- Install (optional): `sudo make install`
- Env (local run): `export LD_LIBRARY_PATH=/usr/lib` and `export GST_PLUGIN_PATH=/usr/lib/gstreamer-1.0`; for DNN: `source $OPENVINO_INSTALLDIR/setupvars.sh`.
- Run tests: after build, execute binaries in `tests/`, e.g. `./tests/test-soft-image --help` or `./tests/test-image-stitching --calib tests/camera_calibration_CamC3C8K.json`.

## Coding Style & Naming Conventions
- C/C++: 4‑space indentation, no tabs, pad operators. Apply with `./tools/pre-commit-code-style.sh` (uses `dos2unix` + `astyle`) or `./code_style.sh` for staged/changed files.
- Filenames: lowercase_with_underscores `.cpp`/`.h`. Mirror header/source names (e.g., `image_processor.cpp` ↔ `image_processor.h`).
- Follow existing patterns in `xcore/*.cpp` for class/method naming and includes order. Keep changes minimal and focused.

## Testing Guidelines
- New tests live in `tests/` as `test-<feature>.cpp`; register in `tests/Makefile.am` (`noinst_PROGRAMS`).
- Prefer self‑contained tests; guard hardware‑specific paths behind flags. Reuse JSON assets in `tests/*.json` when possible.
- Aim to build cleanly with default options and with `--enable-gst --enable-libcl`.

## Commit & Pull Request Guidelines
- Commit subject: `<area>: imperative summary`, e.g., `ocl: fix kernel arg order`.
- Body: what changed and why; note feature flags, perf/ABI impacts, and test coverage. Link issues.
- PRs should: describe motivation, list build options tested, include usage notes for new binaries/plugins, and update docs/tests when applicable.

## Environment & Configuration Tips
- Ensure GPU/OpenCL/Vulkan drivers match selected backends. Missing backends are compiled out if flags aren’t enabled.
- Android builds use `Android.mk`; desktop uses Autotools (`configure.ac`, `Makefile.am`).
