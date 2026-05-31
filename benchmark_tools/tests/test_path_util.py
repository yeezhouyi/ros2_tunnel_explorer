# Copyright 2026 zhouyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for benchmark_tools.path_util pure functions."""

import os
import tempfile

from benchmark_tools.path_util import resolve_goals_path


class TestResolveGoalsPath:
    """Tests for the resolve_goals_path function."""

    def test_absolute_path_returned_directly(self):
        path = "/tmp/some_goals.yaml"
        assert resolve_goals_path(path) == path

    def test_absolute_path_with_existing_file(self):
        with tempfile.NamedTemporaryFile(suffix=".yaml") as f:
            result = resolve_goals_path(f.name)
            assert result == f.name

    def test_relative_path_resolved_to_cwd_when_exists(self):
        with tempfile.NamedTemporaryFile(
            suffix=".yaml", dir=os.getcwd(), delete=False
        ) as f:
            basename = os.path.basename(f.name)
            try:
                result = resolve_goals_path(basename)
                assert result == f.name
                assert os.path.isabs(result)
            finally:
                os.unlink(f.name)

    def test_relative_path_not_found_returns_original(self):
        result = resolve_goals_path("nonexistent_file.yaml")
        assert result == "nonexistent_file.yaml"

    def test_relative_path_in_subdir_not_found_returns_original(self):
        result = resolve_goals_path("config/nonexistent.yaml")
        assert result == "config/nonexistent.yaml"

    def test_empty_string_resolves_to_cwd(self):
        # os.path.join(cwd, "") returns cwd, which exists → resolved.
        result = resolve_goals_path("")
        assert result == os.getcwd()
