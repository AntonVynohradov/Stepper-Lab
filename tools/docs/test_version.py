#******************************************************************************
# Stepper-Lab: Stepper motor model and control simulation (Scilab/Xcos)
#
# Description: Test of project_version() of tools/docs/build_manual.py: the version
#              printed in the manual comes from the git tag.
#                  python tools/docs/test_version.py
#
# Author:      Anton Vynohradov
# Email:       avynohradov@systemfromscratch.com
# License:     Apache License 2.0
# Copyright (c) 2026 Anton Vynohradov
#
# SPDX-License-Identifier: Apache-2.0
#*****************************************************************************

"""Exits with 1 when project_version() gives a wrong value for a state of the repository."""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from build_manual import project_version  # noqa: E402


def git(repo: str, *args: str) -> None:
    subprocess.run(["git", "-c", "user.name=test", "-c", "user.email=test@example.invalid",
                    "-c", "commit.gpgsign=false", "-c", "tag.gpgsign=false", *args],
                   cwd=repo, check=True, capture_output=True)


def commit(repo: str, text: str) -> None:
    with open(os.path.join(repo, "file.txt"), "a", encoding="utf-8") as f:
        f.write(text)
    git(repo, "add", "file.txt")
    git(repo, "commit", "-m", text)


def main() -> int:
    # git makes its object files read-only on Windows
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as base:
        repo = os.path.join(base, "repo")
        os.mkdir(repo)
        not_a_repo = os.path.join(base, "plain")
        os.mkdir(not_a_repo)
        git(repo, "init")
        commit(repo, "one")
        assert project_version(repo) == "development", "no tag"
        git(repo, "tag", "other-tag")
        assert project_version(repo) == "development", "a tag that is no version"
        git(repo, "tag", "v1.0.0")
        assert project_version(repo) == "1.0.0", "at the tag"
        commit(repo, "two")
        value = project_version(repo)
        assert re.fullmatch(r"1\.0\.0-1-g[0-9a-f]+", value), f"after the tag: {value}"
        assert project_version(not_a_repo) == "development", "not a repository"
    print("project_version: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
