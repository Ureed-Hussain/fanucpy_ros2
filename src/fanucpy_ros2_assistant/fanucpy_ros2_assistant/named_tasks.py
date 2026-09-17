# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Load bounded, declarative task files; never execute Python or shell text."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Optional, Tuple

from fanucpy_ros2_task_planner.prompt_parser import (
    CartesianJogPlan,
    PromptParser,
    validate_jog_plan,
)
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    TaskPlan,
    TpProgramPlan,
    validate_cartesian_target_plan,
    validate_tp_program_plan,
)


_NAME = re.compile(r"[a-z][a-z0-9_]{0,63}")
_MAX_BYTES = 16384
_BASE_FIELDS = {"schema_version", "name", "description", "action"}
_JOG_FIELDS = (
    "delta_x_mm", "delta_y_mm", "delta_z_mm",
    "delta_w_deg", "delta_p_deg", "delta_r_deg",
)
_TARGET_FIELDS = ("x_mm", "y_mm", "z_mm", "w_deg", "p_deg", "r_deg")


class TaskFileError(ValueError):
    """Reject an unrecognized task request or invalid task file."""


@dataclass(frozen=True)
class NamedTask:
    """An immutable, single-step task snapshot loaded at assistant startup."""

    name: str
    description: str
    frame_id: str
    plan: Optional[TaskPlan]
    source: str
    sha256: str


@dataclass(frozen=True)
class TaskRequest:
    """A request for the list, a preview, or one exact named task."""

    operation: str
    name: str = ""


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise TaskFileError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def _number(value, field):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise TaskFileError(f"{field} must be a finite number, not text/bool")
    return float(value)


def decode_task(raw: bytes, expected_name: str, source: str) -> NamedTask:
    """Validate the complete file before constructing a supported plan."""
    if len(raw) > _MAX_BYTES:
        raise TaskFileError("Task file exceeds 16 KiB")
    try:
        data = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise TaskFileError(f"Invalid task JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise TaskFileError("A task must be one JSON object")
    if type(data.get("schema_version")) is not int or (
        data["schema_version"] != 1
    ):
        raise TaskFileError("Task schema_version must be integer 1")
    name = data.get("name")
    if not isinstance(name, str) or not _NAME.fullmatch(name):
        raise TaskFileError("Task name must be a lower-case identifier")
    if name != expected_name:
        raise TaskFileError("Task name must match its JSON filename")
    description = data.get("description")
    if (
        not isinstance(description, str) or not description.strip()
        or len(description) > 240
        or any(ord(char) < 32 or ord(char) == 127 for char in description)
    ):
        raise TaskFileError("description must be 1..240 printable characters")
    action = data.get("action")
    fields = set(_BASE_FIELDS)
    frame = ""
    plan = None
    if action in ("cartesian_jog", "cartesian_target"):
        axes = _JOG_FIELDS if action == "cartesian_jog" else _TARGET_FIELDS
        fields.update({"frame_id", "velocity_mm_s", *axes})
    elif action == "tp_program":
        fields.add("program_name")
    elif action != "status":
        raise TaskFileError(
            "Supported actions: status, cartesian_jog, cartesian_target, "
            "tp_program. Scripts, shell commands and multi-step tasks "
            "are not supported."
        )
    if set(data) != fields:
        raise TaskFileError(
            f"Unexpected or missing fields for {action}: "
            f"{sorted(set(data) ^ fields)}"
        )
    if action in ("cartesian_jog", "cartesian_target"):
        frame = data["frame_id"]
        if not isinstance(frame, str) or not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_/]{0,127}", frame
        ):
            raise TaskFileError("A Cartesian task needs an explicit frame_id")
        velocity = data["velocity_mm_s"]
        if type(velocity) is not int or not 0 <= velocity <= 65535:
            raise TaskFileError("velocity_mm_s must be an integer in 0..65535")
        values = [_number(data[field], field) for field in axes]
        plan_type = (
            CartesianJogPlan if action == "cartesian_jog"
            else CartesianTargetPlan
        )
        plan = plan_type(*values, velocity_mm_s=velocity)
    elif action == "tp_program":
        program = data["program_name"]
        if not isinstance(program, str) or not re.fullmatch(
            r"[A-Z][A-Z0-9_]{0,31}", program
        ):
            raise TaskFileError("program_name must be an exact uppercase name")
        plan = validate_tp_program_plan(TpProgramPlan(program))
    return NamedTask(
        name, description.strip(), frame, plan, source,
        hashlib.sha256(raw).hexdigest(),
    )


def validate_task(task: NamedTask, parser: PromptParser, frame_id: str) -> None:
    """Apply local motion limits; execution still needs live driver checks."""
    if isinstance(task.plan, (CartesianJogPlan, CartesianTargetPlan)):
        if task.frame_id != frame_id:
            raise TaskFileError(
                f"Task frame {task.frame_id} differs from {frame_id}; "
                "no coordinate transformation is performed"
            )
    if isinstance(task.plan, CartesianJogPlan):
        validate_jog_plan(
            task.plan, parser.max_translation_step_mm,
            parser.max_rotation_step_deg, parser.max_cartesian_velocity_mm_s,
        )
    elif isinstance(task.plan, CartesianTargetPlan):
        validate_cartesian_target_plan(
            task.plan, parser.max_cartesian_velocity_mm_s
        )


class TaskCatalog:
    """Snapshot only JSON files in one explicitly configured directory."""

    def __init__(
        self, directory: str, *, allow_external_symlinks: bool = False
    ):
        root = Path(directory).expanduser().resolve(strict=True)
        if not root.is_dir():
            raise TaskFileError("task_directory must be a directory")
        tasks = {}
        for index, path in enumerate(sorted(root.glob("*.json"))):
            if index >= 64:
                raise TaskFileError("At most 64 task files are supported")
            if not _NAME.fullmatch(path.stem):
                raise TaskFileError(f"Invalid task filename: {path.name}")
            # Custom directories do not follow task symlinks outside the
            # selected directory. The package's built-in directory may opt
            # in because ``colcon build --symlink-install`` deliberately
            # links installed data files back to the workspace source tree.
            resolved = path.resolve(strict=True)
            if (
                (resolved.parent != root and not allow_external_symlinks)
                or not resolved.is_file()
            ):
                raise TaskFileError(
                    "Task must be a file inside task_directory"
                )
            with resolved.open("rb") as stream:
                raw = stream.read(_MAX_BYTES + 1)
            tasks[path.stem] = decode_task(raw, path.stem, str(resolved))
        self._tasks = tasks

    @property
    def names(self) -> Tuple[str, ...]:
        """Return the available exact task identifiers."""
        return tuple(self._tasks)

    def get(self, name: str) -> NamedTask:
        """Look up an identifier, never a user-provided path."""
        if name not in self._tasks:
            raise TaskFileError(f"Unknown task {name!r}; use /tasks")
        return self._tasks[name]


def _task_words(name: str) -> str:
    value = name.removesuffix(".json").replace("_", " ")
    value = re.sub(r"\b(task|file|example|demo)\b", " ", value)
    return " ".join(value.split())


def parse_task_request(prompt: str, names: Tuple[str, ...]):
    """Match friendly wrappers without guessing names or accepting modifiers."""
    text = " ".join(prompt.lower().strip().split()).rstrip(".!?")
    if text in {
        "/tasks", "tasks", "list tasks", "list examples", "show examples",
        "show me the examples", "show me the tasks", "what tasks can i run",
        "what examples can i run", "which examples can i run",
    }:
        return TaskRequest("list")
    match = re.fullmatch(
        r"(?:(?:please|can you|could you|would you|can u|could u)\s+)*"
        r"(?P<verb>/run|/preview|run|start|execute|preview)\s+"
        r"(?P<name>.+?)(?:\s+for me)?(?:\s+please)?", text
    )
    if match:
        target = re.sub(r"^the\s+", "", match["name"])
        key = _task_words(target)
        matches = [name for name in names if _task_words(name) == key]
        if len(matches) == 1:
            return TaskRequest(
                "preview" if "preview" in match["verb"] else "run",
                matches[0],
            )
        # Preserve existing exact TP-program wording, not scripts or task paths.
        if re.fullmatch(r"(?:tp\s+)?program\s+[a-z][a-z0-9_]{0,31}", target):
            return None
        raise TaskFileError(
            "Please choose one exact task from /tasks. I cannot run "
            "arbitrary files, guess a name, combine tasks, or change "
            "a saved task's speed/target from a prompt."
        )
    if re.search(r"\b(run|execute|start)\b", text) and (
        re.search(r"\b(task|file|example|demo)\b|\.(json|py|sh)\b", text)
        or any(name in text for name in names)
    ):
        raise TaskFileError(
            "Nothing was sent. For one saved task use 'run task NAME'; "
            "for advice use '/ask your question'."
        )
    if (
        any(name in text for name in names)
        or re.search(r"\.(json|py|sh)\b", text)
        or re.search(r"\b(task|file|example|script)\b", text)
    ):
        raise TaskFileError(
            "For package help use /ask; for a saved task use /tasks, "
            "'preview NAME' or 'run task NAME'. Files are not passed to "
            "the movement interpreter."
        )
    return None
