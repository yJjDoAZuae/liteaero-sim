# Notebook-cleaning pre-commit hook

[clean_notebooks.py](clean_notebooks.py) and [pre-commit](pre-commit) keep
committed Jupyter notebooks free of transient, run-dependent data.

## What gets stripped

Per staged `*.ipynb`, the cleaner removes everything that is a product of
*running* the notebook rather than part of its source:

- code-cell `outputs` (cleared to `[]`)
- code-cell `execution_count` (reset to `null`)
- transient per-cell metadata (`execution`, `ExecuteTime`, `collapsed`, `scrolled`)
- notebook-level kernel identifiers (`kernelspec`, `language_info` — the
  previously-used kernel name and interpreter version)

Files are rewritten in Jupyter's canonical on-disk form (`indent=1`, sorted
keys, trailing newline), byte-identical to what Jupyter itself saves, so there
is no diff churn. An already-clean notebook is left untouched.

The cleaner depends only on the Python standard library (a notebook is just
JSON), so it runs under any interpreter.

## Install

The hook is checked into the repo but `.git/hooks/` is not version-controlled,
so install it once per clone, from the repo root:

```sh
cp python/scripts/pre-commit .git/hooks/pre-commit
chmod +x .git/hooks/pre-commit
```

On commit, the hook cleans every staged notebook and re-stages it. It discovers
a Python interpreter automatically, preferring an in-repo virtual environment
(`python/.venv/Scripts/python.exe` or `.venv/...`) and falling back to
`python3` / `py` / `python` on `PATH`. If a notebook is staged but no
interpreter is found, the commit is blocked with a clear message.

## Manual use

Clean notebooks directly, without committing:

```sh
python python/scripts/clean_notebooks.py path/to/notebook.ipynb [more.ipynb ...]
```

## Caveat

A notebook that is only partially staged (some hunks staged, others not) is
re-staged in full, because the hook cleans the working-tree file. Stage
notebooks completely before committing, or run the cleaner manually.
