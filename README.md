# A Stereo Vision Matching Library

This project is a C/C++ stereo vision library built using Conan 2, featuring modern C++ standard support:
epipolar rectification, SGBM matching, disparity-to-depth conversion and point cloud export.

## Badges

![License](https://img.shields.io/github/license/HeT-FTI/fcpp?color=blue&label=license)
![C++](https://img.shields.io/badge/C%2B%2B-17%2F20%2F23-blue?logo=c%2B%2B&logoColor=white)
![Modules](https://img.shields.io/badge/modules-C%2B%2B23%20experimental-purple?logo=c%2B%2B&logoColor=white)
![CI](https://github.com/HeT-FTI/fcpp/actions/workflows/ci-orchestrator.yml/badge.svg)
![Release](https://github.com/HeT-FTI/fcpp/actions/workflows/semver-release.yml/badge.svg)
![CMake](https://img.shields.io/badge/cmake-3.28%2B-orange?logo=cmake)
![Python](https://img.shields.io/badge/python-3.10%2B-blue?logo=python)
![Doxygen](https://img.shields.io/badge/docs-Doxygen-green?logo=doxygen&logoColor=white)
![Sphinx](https://img.shields.io/badge/docs-Sphinx-blue?logo=readthedocs&logoColor=white)

## Project Overview

- **Language**: C/C++
- **Library Build System**: Python with Conan 2.0
- **File Build System**: Doxygen, Graphviz, Sphinx
- **Module Support**: Optionally activated, when C++ standard ≥ 23
- **Metadata-Driven**: `metadata.json` is the single source of truth (build, deps, docs, CI)
- **Skills & Agent**: harness-agnostic Agent Skills + one routing agent (canonical `.github/skills/`,
  default `.agents/skills` view; optional `.claude`/`.codex` views)
- **Pipeline**: left/right images + stereo calibration + JSON parameters → disparity map, depth map,
  PCD/PLY point cloud
- **Component Structure**:
    - pairwise header and source assumption
    - suffix distinguishment, (.h, .c) for C part, and (.hpp, .cpp) for C++ part
    - documenting system uses .dox for pure docstring, .cxx for examples codes; both are
      documentation-only and live exclusively under `docs/doxygen/dox/`, never in `include/`/`src/`

## Features

- Conan-based modern dependency management (OpenCV pinned to the modules actually used)
- Supports C++ standards 17, 20, and 23
- Automatic module file generation (`.ixx`/`.cppm`) from headers/sources
- Cross-platform compatibility (Windows, Linux, macOS)
- Dual C and C++ interfaces with separate linkage targets
- Doxygen annotation support for object exporting (`@exporter`, `@attacher`)
- Automation documenting system via Doxygen and Sphinx
- `metadata.json` as the single source of truth (build / deps / docs / CI)
- Harness-agnostic Agent Skills (`.github/skills/`, 15 task skills + one routing agent) with a default
  `.agents/skills` view and optional `.claude`/`.codex` views
- Metadata + gitmoji driven CI/CD orchestration (GitHub Actions)
- Agentic Coding workspace (`/workspace/`, git-ignored) — also holds the CLI consumer
- Epipolar rectification with common-ROI cropping, rebased `Q`/`P` and a validity mask from remapping
  a white image
- `cv::StereoSGBM` wrapper: padding, 1/16 px fixed-point unpacking, search-window clipping and a
  sub-pixel left-right consistency check
- Depth from `Z = fx·|Tx| / (d − doffs)`, so a principal-point offset no longer skews the cloud
- Point cloud in the rectified left camera frame, geometry only (`x y z`, no RGB); PCD and PLY
  writers (ascii or binary) without PCL
- Interface contract: images enter through `set_input_image()`, results leave through `get_*` and
  `save_*`; `run()` computes and writes nothing
- `save_disparity_image()` / `save_depth_image()` take `colorize`: the default keeps the measurement
  encoding (1/16 px png, millimetre png, float tif), `true` writes an 8-bit `COLORMAP_TURBO` view
  stretched over the 5th–95th percentile, with invalid pixels black and near warm in both maps

## Agentic Coding Workspace

The template reserves `/workspace/` (declared in `.gitignore`) for agent-generated work products produced
during Agentic Coding sessions — e.g., implementation plans, test contracts, audit reports, and other
intermediate artifacts. Keep such working files under `/workspace/` so they never pollute the tracked
source tree.

This project additionally keeps the command-line consumer in `workspace/tools/stereo_cli/`: it is a thin
`Pipeline` wrapper, so it belongs to the working set rather than to the shipped library.

## Skills & Agent (any harness)

The template ships a library-level Agent Skills system — **15 task skills plus one routing agent**
(`het-agent`), canonical under `.github/skills/`. The same bodies are exposed to each harness through
generated views, never copies. Default `sync` creates **one root directory** (`.agents/`); Claude Code
and Codex views are optional:

| Harness | Native view | How to invoke | Default? |
|---|---|---|---|
| DSH | `.agents/skills` | `skill` tool with the exact name, or `/het-build` | yes |
| Cursor | `.agents/skills` | `/het-build` or `@het-build` | yes |
| Gemini CLI | `.agents/skills` alias | automatic `activate_skill` | yes |
| GitHub Copilot | `.github/skills` (VS 2026 also `.agents`) | `/het-build` in chat | canonical |
| Claude Code | `.claude/skills` | `/het-build` | optional (`sync --harness claude`) |
| Codex | `.codex/skills` | `/use het-build` | optional (`sync --harness codex`) |
| Any other harness | direct read | read `.github/skills/<name>/SKILL.md` | — |

In this template repository the canonical tree is `.github/skills/` and the view is a
symlink, so materialize it once after cloning:

```bash
python .github/skills/_shared/skillkit/cli.py sync
python .github/skills/_shared/skillkit/cli.py validate --strict
```

A repository **created from this template** needs none of that: the `Template Bootstrap`
workflow commits a byte-identical `.agents/skills` view right after instantiation, so
`git clone` → open in DSH/Cursor/Gemini works immediately. The same commit drops this
repository's own `CHANGELOG.md`, because its entries link here; the instance rebuilds it
on its first release.

`skillkit` refuses to let a view drift: a link must point at the canonical tree and a
copy must be byte-identical, so `.github/skills/` stays the only body you edit.
There are **no root-level `AGENTS.md`/`CLAUDE.md`/`GEMINI.md` files**: the only default root addition is
the git-ignored `.agents/` directory.
Examples: `het-guide` for onboarding, `het-testgen` to generate tests, `het-agent` for composite tasks
(e.g. *"add a module, test it, and commit"*). See `.github/skills/README.md`,
`.github/skills/_shared/routing.md`, and `.github/skills/_shared/harness-adapters.md`.

## Build Requirements

- Python 3.10+ (Conan tooling)
- Conan 2.0+
- Compatible C/C++ compiler:
    - GCC
    - Clang
    - MSVC
- CMake (auto-installed by Conan)
- OpenCV 4.14.0 — pulled by Conan, no system install needed. The recipe pins
  `highgui=False`, `with_ffmpeg=False` and `with_vaapi=False`: none of the three is needed by
  `core`/`imgproc`/`imgcodecs`/`calib3d`, and each one otherwise turns a host build into a
  system-package hunt (for example `vaapi/system` fails outright without `libva-dev`)

## Documenting Requirements

- Doxygen
- Graphviz
- sphinx
- sphinx-intl
- sphinx-rtd-theme

## Test Requirements

- GTest

## CI/CD (metadata-driven)

All GitHub Actions workflows are orchestrated from `metadata.json` (entry: `ci-orchestrator.yml`, decision:
`metadata-controller.yml`). Each pipeline is gated by a `workflow_triggers.*` switch and triggered by a
gitmoji in the commit message. The canonical commit form is `<type>(<emoji>): <description>` — the emoji
sits in the parentheses right after the commit word (e.g. `feat(:fire:): ...`, `chore(:package:): ...`);
as a **soft rule** the emoji also triggers from anywhere in the message:

| Pipeline | gitmoji in commit message | Gate (`metadata.json`) |
|----------|---------------------------|------------------------|
| Build | `:building_construction:` | `workflow_triggers.build` |
| Tests + coverage | `:beer:` | `trigger_tests` / `activate_code_coverage` |
| Release | `(:package:):` | `workflow_triggers.release` + the gitmoji in the pushed commits |
| Docs | `:book:` | `workflow_triggers.docs` |
| Security / lint | `:shield:` | `workflow_triggers.security_scan` (also runs on every PR) |
| Online cross-compile | `:hammer_and_wrench:` | `workflow_triggers.cross_compile` (Conan cross-build from a generated profile; decoupled: the emoji starts only this one) |
| Board cross-build | `:fire:` (or `🔥`) | hetai self-hosted runner |

> **Note**: `workflow_triggers.build` / `.tests` / `.docs` / `.security_scan` are
> enabled by default (commit-lint & schema gates always run on push/PR; build/tests/security
> shift-left on PRs).
> `release` needs both halves: the switch is the standing permission, the gitmoji is this push's
> intent. `build_type` is the project's default build type and gates nothing.
>
> **This project turns `cross_compile` off** (`workflow_triggers.cross_compile=false`): the library
> depends on OpenCV, which is not in `baremetal_white_list`, so a baremetal cross-build cannot resolve
> its dependencies.

## Crash Course of Build

### 1. Build then test your library

inplace build and test

```bash
conan create . -s build_type=Release --build=missing
```

The first run compiles OpenCV from source (ConanCenter ships no binary for every toolchain/compiler
combination), which takes about 5–10 minutes; later runs hit the cache. `metadata.json` declares
`build_type: Release` because the pinned OpenCV build also is Release — a Debug run would rebuild
OpenCV and roughly double the wait.

`conan create` also runs `test_package/main.cpp`, which is a complete example rather than a stub: it
loads the BlenderProc oven scene shipped in `test_package/resources/oven/` (a 640×480 pair, its
calibration as `stereo.yml` plus per-camera `left_mono.yml`/`right_mono.yml`, and
`sgbm_config.json`), rectifies, matches, and prints the report before writing five artefacts into
`example_output/` next to the binary: `disparity.png` and `depth.png` in their measurement encodings,
`disparity_color.png` and `depth_color.png` as viewable thermal ramps, and `cloud.pcd`. The
scene's own ground truth puts the median disparity at 73.26 px and the median depth at
0.1129 m, so the program exits non-zero when the result drifts away from that.

`main.cpp` is a program, not a GTest binary, so it is **not** a ctest case: `test_package/conanfile.py`
runs it in `test()`, echoes every line with an `[example]` prefix, and copies the same text to
`test_package/test/export/ExampleResult.log`. The program is executed before ctest — its own
coverage counters have to exist by the time the report is collected — but its output is replayed at
the very end, so a `conan create` log finishes with

```
[example] result of test_package/main.cpp
[example] rectified 640x480
...
[example] saved     disparity.png, depth.png (measurement) and disparity_color.png, depth_color.png (colour), plus cloud.pcd in example_output
[example] elapsed   load 14.7 ms, run 271.5 ms, write 136.0 ms, total 423.2 ms
[example] memory    peak 34.1 MB resident
```

instead of with the coverage table. The example also reports cost: `elapsed` splits the wall clock
into load (I/O for images and calibration), run (the algorithm) and write (the five artefacts), which
frames the finer per-stage `timing` line above it; `memory` is the peak resident set — `/proc` `VmHWM`
on Linux, `ru_maxrss` on macOS, the peak working set on Windows. All three start at zero in this
process, which matters: Linux's `ru_maxrss` is inherited across `fork` and not reset by `exec`, so
reading it instead reported 72 MB for a run that truly peaks at 34 MB whenever the parent was heavy
(conan's Python). The values agree with `/usr/bin/time -v` to within 1 MB.

Look for that prefix, or run the binary directly from
`test_package/build/<profile>/`; `ctest -N` will never list it. Two switches keep their distance:
`trigger_tests` in `metadata.json` governs the 64 GTest cases and the coverage report, while the
example runs whenever the test package is built for the host (`can_run`), regardless of that switch.
The example's exit code becomes the recipe's verdict, so a red example fails `conan create` with
`test_package/main.cpp exited with code N` rather than a bare build error; a red example also skips
the suite, exactly as an exception from `test()` did before.

cross-build to host device (assume toolchain and profile are ready):

```bash
conan create . -pr:b=default -pr:h=arm_profile -s build_type=Release --build=missing -tf=""
```

> Not supported for this project: OpenCV is not baremetal-cross-compilable here, and
> `workflow_triggers.cross_compile` is off. The command above only documents the template contract.

### 2. Build documentations

```bash
python ./docs/build.py
```

### 3. Full local pass

Steps 1-2 end to end, then drop the local package. `build_type` is the one `metadata.json` declares,
and the package name is yours — there is nothing to edit here:

```bash
conan create . -pr:b=default -pr:h=default -s build_type=Release --build=missing
python ./docs/build.py
python ./test_package/conanfile.py
conan remove "stereo_vision/*" --confirm
```

### 4. Add requirements

Add your required libraries in *conandata.yml* where dependency graph is automatically computed from, then 
modify the **dependencies** field in *metadata.json* to config proper package names and associated targets to 
link (no need modification on *CMakeLists.txt*).

Requirements for your project can be the package archived on [Conan Center](https://conan.io/center), or user 
built ones. If the later one, at least you need a locale Conan server for managing your libraries.

This project uses one entry only — `opencv/4.14.0`, mapped to the aggregate CMake target
`opencv::opencv` in the `cpp` bucket. A new C/C++ dependency whose headers `src/` includes also needs a
line in the `mapping` table of `.github/workflows/security-linters.yml`, otherwise the clang-tidy gate
fails with a file-not-found (that failure is intentional).

### 5. Run MegaLinter locally (blocking SAST)

MegaLinter mirrors the CI `megalinter` job (`.github/workflows/security-linters.yml`): same config
(`.github/misc/.mega-linter.yml`), same full image, and the same version (`v8.8.0`, pinned to match the
CI action). It runs the SAST layer — gitleaks / semgrep / checkov / devskim, plus clang-format
& cppcheck. The **required** gates (clang-format / clang-tidy / gitleaks with pinned tool versions) are the
native `quality-gates` job of the same workflow.

Prerequisites: Docker (or Podman) + Node.js ≥ 20.

```bash
bash .github/misc/run-megalinter.sh             # lint the whole codebase
bash .github/misc/run-megalinter.sh --fix       # auto-apply fixes (e.g. clang-format)
bash .github/misc/run-megalinter.sh src/foo.c   # lint selected files only
CONTAINER_ENGINE=podman bash .github/misc/run-megalinter.sh   # Podman users
```

```bash
pip install "clang-format==23.1.0" clang-tidy==19.1.0
python3 .github/misc/gates.py format
GATES_INCLUDE=<opencv-prefix>/include/opencv4 python3 .github/misc/gates.py tidy
python3 .github/misc/gates.py guard
```

`GATES_INCLUDE` is only needed outside CI: CI obtains the OpenCV headers from apt via the
`conandata.yml` → `mapping` table.

- The first run pulls the `v8.8.0` image (a few GB — cached on later runs).
- Reports are written to `megalinter-reports/` (git-ignored); per-linter details are in
  `megalinter-reports/linters_logs/ERROR-*.log`.
- The runner (`mega-linter-runner`) is a devDependency — `npm ci` installs it, or it is fetched
  automatically on first use.

## All-in-one Project Structure

```
project-root/
├── conanfile.py              # Conan recipe
├── CMakeLists.txt            # CMake build framework
├── metadata.json             # Project metadata configuration (single source of truth)
├── conandata.yml             # Dependency specifications, Conan plugin support
├── package.json              # Node-side tooling deps (commitlint / semantic-release / ajv)
├── Doxyfile                  # Doxygen entry (hard-referenced by docs/build.py)
├── LICENSE                   # Apache v2 Project license
├── NOTICE                    # Notice file of Apache v2
├── .vscode/                  # Recommended editor config (extensions / settings / tasks)
├── .agents/skills            # Default generated view (git-ignored) → .github/skills
├── .claude/skills            # Optional generated view (git-ignored) → .github/skills
├── .codex/skills             # Optional generated view (git-ignored) → .github/skills
├── .devcontainer/            # Reproducible dev environment (Dockerfile)
├── .github/                  # CI/CD + canonical agent skills
│   ├── workflows/            # ci-orchestrator / metadata-controller / build / test / docs / release / security
│   ├── skills/               # canonical: 15 task skills + het-agent + _shared + manifest.json
│   │   └── _shared/skillkit/ # validate / sync / catalog tool
│   ├── misc/                 # clang-format(-c/cpp) / clang-tidy / gitleaks / releaserc / commitlint /
│   │                         # metadata.schema / mega-linter / pre-commit / labels / Codegen-Starter.txt
│   ├── CODEOWNERS            # Required reviewers
│   ├── CONTRIBUTING.md       # Contribution & commit conventions
│   ├── dependabot.yml        # Grouped monthly dependency updates
│   └── PULL_REQUEST_TEMPLATE.md
├── include/                  # Public headers
│   └── sv_*.hpp              # C++ interface headers (pairwise with src/sv_*.cpp)
├── src/                      # Implementation files
│   ├── sv_*.cpp              # C++ sources
│   └── *.ixx/*.cppm          # Auto-generated Module files (in experimental)
├── benchmark/                # Cross-compile & on-board benchmark framework (kept, unused here)
├── .hetai/                   # hetai package matrix (cross-compile targets)
├── workspace/                # Agentic Coding work products (git-ignored)
│   ├── stereo-migration-plan.md  # Implementation plan and result log
│   └── tools/stereo_cli/     # Command-line consumer of the packaged library
├── docs/                     # Documentations root
│   ├── doxygen/              # Doxygen system main root
│   │   ├── dox/              # Pure documentations' folder (the only home of .dox/.cxx)
│   │   │   ├── demos/        # Examples catalogue (hand-written, standalone)
│   │   │   │   ├── *.dox     # Documenting docstring
│   │   │   │   └── *.cxx     # Example codes
│   │   │   └── *.dox         # Main pages and etc
│   │   └── ...
│   ├── sphinx/               # Sphinx system main root
│   │   ├── source/           # Source files of sphinx system
│   │   ├── locales/          # Pot files for internalization
│   │   └── ...
│   └── images/               # Static images for doxygen/sphinx system
└── test_package/             # Test project
    ├── export/               # Log for testing results
    ├── resources/            # Test resources: unit fixtures plus the oven/ example dataset
    ├── test/
    │   ├── unit/             # Standing unit tests (coverage target)
    │   ├── stress/           # GTest stress / performance tests
    │   └── python/           # Optional python consumer (bindings enabled)
    ├── main.cpp              # Validation program for package
    ├── conanfile.py          # Conan recipe for test_package
    └── CMakeLists.txt        # CMake build workflow for test_package
```

> **Suffix rule**: `.dox` and `.cxx` are **documentation-only**. They live exclusively under
> `docs/doxygen/dox/` as hand-written standalone files (`mainpage.dox`, `demos/*.dox`,
> `demos/*.cxx`) and are never written as a docstring inside `include/` or `src/`, which carry
> `.h/.c/.hpp/.cpp` only. A complete doc set is a landing page plus a tutorial; example code is
> optional (0..N `.cxx` per `.dox`, no pairing) and `docs/build.py` fails the build when any of
> those properties breaks.

> **Module pairing**: every `include/sv_x.hpp` has exactly one `src/sv_x.cpp`, and each pair shares one
> namespace (`sv`). The C component of this package has no sources and CMake generates a placeholder TU
> for it, so `libstereo_vision_c` stays linkable.

## Module Generation (experimental)

When `generate_modules_inplace` is enabled in `metadata.json`:

1. Header/source pairs automatically generate module files
2. `#include` directives are converted to `import` statements
3. Doxygen annotations control symbol visibility:
    - `@exporter`: Exports symbols in modules
    - `@attacher`: Attaches symbols to modules

This feature is experimental now, however, the specific syntax can make the existing project a ease 
migration to fit the future C++ standard.

## Compiler Support Matrix

This states what CI **actually builds and runs**, not what the language permits. The runner labels live
in `.github/workflows/`; `build_type` / `build_cppstd` / `build_cstd` live in `metadata.json`.

| Feature          | MSVC (`windows-latest`) | Apple-Clang (`macos-latest`) | GCC (`ubuntu-latest`) |
|------------------|-------------------------|------------------------------|-----------------------|
| Build, link, run | ✓                       | ✓                            | ✓                     |
| C Compatibility  | ✓                       | ✓                            | ✓                     |
| Automatic Export | ✓                       | ✓                            | ✓                     |

**C++ Modules are experimental and are not exercised by CI**: `generate_modules_inplace` is `false`, so
no leg compiles a module interface. Treat the feature as unverified until a leg covers it.

## Platforms Support

- **Desktop**: Windows, Linux, macOS — each is a build-and-run CI leg
- **Embedded / cross-compiled**: **not supported for this project**. The pipeline needs OpenCV's
  `calib3d`/`imgcodecs`, which are not cross-compilable to a baremetal MCU, and `cross_compile` is off in
  `metadata.json`. `.github/workflows/cross-compile.yml` remains available for template-derived projects
  that do not need OpenCV.

## Downstream Contract (HeT DevTools)

This template is also built by the **HeT DevTools** VS Code extension in a *managed lane*: it creates its
own private venv / `CONAN_HOME` and a **generated** Conan profile instead of probing the user's machine.
The interfaces below are **backward compatible** — by default the behaviour is byte-for-byte the CI one.

| Interface | Value | Default | Meaning |
|---|---|---|---|
| `HET_CMAKE_MIN` | version string (e.g. `3.15`) | unset | Overrides the `cmake_minimum_required` floor (CI baseline is 3.28) |
| `HET_CMAKE_BUILD_REQUIRE` | `none` or a version | unset | Skips / replaces the ConanCenter `cmake/<metadata.cmake_version>` (read by **both** recipes) |
| `metadata.graphviz_bin` | directory path | **absent** | Absent = use `dot` from `PATH`. This is a **machine-specific** field and normally should not be committed |
| `conancenter` remote name | keep the standard name | — | A downstream private `CONAN_HOME` may point it at a corporate mirror — never assume its URL |
| Coverage artifact | `test_package/test/export/coverage/coverage_summary.json` | — | The JSON and `coverage_report/index.html` are rendered from the **same merged tracefile**, so their rates agree; parse the JSON because the HTML layout belongs to genhtml. Floors live in `metadata.coverage.floors.<RUNNER_OS>` (70% for this project) — see `metadata-contract.md` for how to read and change them. CI publishes the lcov leg as the canonical `Coverage-report` artifact and keeps `Coverage-report-<os>-<ts>` as per-leg evidence |
| `user.het:run_tests` | conan conf (`-c user.het:run_tests=False`) | `True` | Narrows `metadata.trigger_tests` for one run and leaves the library uninstrumented — what the build-only CI legs do. Namespaced to the template, not to the package, so a rename never has to touch it. **This project folds it into `package_id`**: an instrumented archive carries `__gcov_*` symbols that a plain consumer cannot link, so the two variants must be different packages |

(The implementations live in `CMakeLists.txt`, `conanfile.py`, `test_package/conanfile.py` and `docs/build.py`.)

## Benchmark (on-board)

`benchmark/` cross-compiles the library to real hardware and measures performance on-board:

- **Cortex-M (baremetal)**: flash via JLink / OpenOCD / PyOCD, timing via SYSTICK
- **Cortex-A (Linux)**: deploy via ADB / SSH, timing via `clock_gettime`

Edit `benchmark/platform/bench_config.json` (board parameters) and `benchmark/bench_entry.cpp` (algorithm
cases), then run `python benchmark/script/run_bench.py` (add `--no-flash` to build only). Results follow
the `BENCHMARK_START / RESULT|name|cycles / BENCHMARK_END` protocol. See `benchmark/README.md`.

> Role split: `test_package/` runs **host-side** GTest verification (desktop/x86_64, even when the library
> is cross-compiled to baremetal), while `benchmark/` runs **target-side** on-board verification.
>
> **Not enabled for this project**: the framework is kept as the template provides it, but the library
> needs OpenCV and `baremetal_white_list` is empty, so no board leg currently builds.

## Commit Convention & Versioning

Commit style uses the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
specification, extended with the template's private **emoji superset** (see the CI/CD table above) so a
single message both bumps the version and triggers the right pipeline.

**Canonical commit form** — the emoji is placed in the parentheses right after the commit word:

```
<type>(<emoji>): <description>
```

Examples: `feat(:fire:): cross-compile support`, `test(:beer:): vector add cases`,
`chore(:package:): prepare release`. As a soft rule, the emoji anywhere in the message also triggers the
pipeline, but the parenthesized placement is the recommended convention.

Versioning is driven by **semantic-release** (`semver-release.yml` + `.github/misc/.releaserc.json`): the commit prefix
decides the jump — `feat` → minor, `fix`/`perf` → patch, `BREAKING CHANGE`/`!` → major. A release
generates `CHANGELOG.md` and rewrites the `version` in `metadata.json`. (The legacy
commit-base-versioning mechanism has been removed.)

## To Do Things

- MSVC has no coverage backend configured, so the test + coverage chain runs on `ubuntu-latest` and
  `macos-latest` only (`.github/workflows/full-test-automation.yml`). A Windows leg needs MSVC
  instrumentation first.
- `binarysgbm` (Census cost via `opencv_contrib/xstereo`) and WLS filtering
  (`ximgproc`, `createDisparityWLSFilter`) are not ported: both live in `opencv_contrib`, which the Conan
  package does not build.
- JPEG input is lossy; precision evaluation should use PNG. The library does not warn about it.

## License

[Apache-2.0] - See included LICENSE file for details.
