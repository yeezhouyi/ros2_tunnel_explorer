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

"""Pure path resolution helpers for benchmark tools (no ROS dependencies)."""

import os


def resolve_goals_path(goals_arg):
    """
    Resolve a goals-file path from a user-supplied argument.

    Rules
    -----
    - Absolute path → returned directly.
    - Relative path → resolved against ``os.getcwd()``.  If the resolved path
      exists, it is returned; otherwise the original argument is returned
      (the caller handles the "not found" error).

    Parameters
    ----------
    goals_arg : str
        Path as supplied by the user (or default).

    Returns
    -------
    str
        Resolved absolute path, or the original argument if resolution fails.
    """
    if os.path.isabs(goals_arg):
        return goals_arg
    resolved = os.path.normpath(os.path.join(os.getcwd(), goals_arg))
    if os.path.exists(resolved):
        return resolved
    return goals_arg
