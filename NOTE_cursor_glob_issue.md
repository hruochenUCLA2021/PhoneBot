# NOTE: Cursor Glob/Grep Index Visibility Issue

## Context

In this workspace, Cursor agent tools can read files by exact path, but cannot discover/search many files under sample folders with `Glob`/`Grep`.

Observed on:

- Workspace: `/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot`
- Problem folders:
  - `reference_repo/mediapipe-samples`
  - `example_code/mediapipe-samples`


## Symptom Summary

### What works

- Direct file reads by absolute path work:
  - example: `.../example_code/mediapipe-samples/examples/llm_inference/js/README.md`
  - example: `.../example_code/mediapipe-samples/examples/pose_landmarker/android/app/src/main/AndroidManifest.xml`
- Shell listing works (`ls` shows files/folders exist).

### What fails

- `Glob` on those folders returns `0 files found` for patterns like:
  - `**/*.kt`
  - `**/*.md`
  - exact file path pattern like `**/example_code/mediapipe-samples/examples/llm_inference/js/README.md`
- `Grep` on those folders returns no matches even for known strings.

### Important contrast

- Root-level search can still return some files (e.g., root `README.md`), so tools are not completely dead.
- The issue is selective subtree invisibility.


## What Was Tried

1. Updated `.gitignore` to unignore folders.
2. Removed nested `.git` in sample repo copy.
3. Reopened Cursor multiple times.
4. Verified folders are normal directories (not symlinks).
5. Tested both broad and exact-path glob patterns.

Result: problem persisted for `Glob`/`Grep`, while direct path reads continued to work.


## Likely Cause

Most likely Cursor indexing/search scope issue (stale or constrained index), not Ubuntu/Python/venv issue.

Reasons:

- Filesystem access is valid (shell and direct read succeed).
- Only index-dependent tools (`Glob`/`Grep`) fail on specific trees.
- Behavior matches reports of ignored/excluded or large/vendor trees not being traversed by index/search.


## Not the Cause

- Not Python `venv`.
- Not Linux/Ubuntu `glob` package behavior.
- Not basic filesystem path problems.


## Reproduction Steps (shareable)

1. Put a known file at:
   - `example_code/mediapipe-samples/examples/llm_inference/js/README.md`
2. Run Cursor `Glob`:
   - target directory: workspace root
   - pattern: `**/example_code/mediapipe-samples/examples/llm_inference/js/README.md`
3. Observe: `0 files found`
4. Read file by exact absolute path with file-read tool.
5. Observe: content is returned successfully.


## Practical Workaround

- Use exact absolute path reads for files inside affected folders.
- If searchable discovery is needed, copy the needed sample subset into a folder that Cursor definitely indexes (project-local small folder).


## References (public discussion/docs)

- Cursor forum: rg/glob failing on gitignored paths  
  https://forum.cursor.com/t/rg-and-glob-tools-failing-on-directory-search-for-gitignored-paths/152094
- Cursor forum: search in .gitignored folders  
  https://forum.cursor.com/t/cursor-doesnt-search-in-gitignore-d-folders/68115
- Cursor ignore docs  
  https://cursor.com/docs/reference/ignore-file

