"""Low-rate Jev router and semantic sentinel for a pi0.5 control loop.

Jev never bypasses the local safety layer.  Missing safety state, API failures,
invalid responses and uncertain routes all fail closed to ``hold``.
"""

from __future__ import annotations

import argparse
import dataclasses
import getpass
import json
import os
from pathlib import Path
import urllib.error
import urllib.request


API_URL = "https://api.typesafe.ai/v1/systemone"
DEFAULT_MODEL = "jev-1.13.0"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "configs" / "jev.local.json"
DEFAULT_ROUTES = {
    "continue": "Continue the current bounded subtask with a new pi0.5 action chunk.",
    "replan": "Generate a new pi0.5 action chunk for the current subtask from fresh observations.",
    "recover": "Enter the predefined bounded recovery state before resuming the task.",
    "hold": "Hold position and wait for a fresh observation or operator decision.",
    "stop": "Stop autonomous execution because continuing may be unsafe.",
    "complete": "The requested task is complete; generate no more actions.",
}


@dataclasses.dataclass(frozen=True)
class SupervisorDecision:
    route: str
    generator_enabled: bool
    source: str
    reason: str
    route_confidence: float | None = None
    hazard_probability: float | None = None
    model: str | None = None

    def as_dict(self):
        return dataclasses.asdict(self)


class JevSupervisor:
    """Evaluate structured state without participating in the servo loop."""

    def __init__(
        self,
        api_key: str,
        *,
        model: str = DEFAULT_MODEL,
        timeout_s: float = 2.0,
        route_confidence_min: float = 0.75,
        hazard_stop_threshold: float = 0.70,
    ):
        if not isinstance(api_key, str) or not api_key.strip():
            raise ValueError("TypeSafe API key is empty")
        if not isinstance(model, str) or not model.strip():
            raise ValueError("TypeSafe model is empty")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if not 0 <= route_confidence_min <= 1 or not 0 <= hazard_stop_threshold <= 1:
            raise ValueError("confidence thresholds must be in [0, 1]")
        self.api_key = api_key.strip()
        self.model = model
        self.timeout_s = timeout_s
        self.route_confidence_min = route_confidence_min
        self.hazard_stop_threshold = hazard_stop_threshold

    def evaluate(self, state, routes=None):
        try:
            routes = dict(DEFAULT_ROUTES if routes is None else routes)
        except (TypeError, ValueError):
            return self._hold("local", "candidate routes must be a mapping")
        fallback = self._validate_inputs(state, routes)
        if fallback is not None:
            return fallback

        payload = {
            "state": state,
            "model": self.model,
            "questions": {
                "next_route": {
                    "type": "choice",
                    "instructions": {
                        "question": "Which candidate route should the robot supervisory state machine select next?",
                        "rules": [
                            "Use only the structured state supplied in this request.",
                            "Prefer hold or stop when observations are stale, contradictory, or insufficient.",
                            "Do not assume this decision can override deterministic local safety checks.",
                        ],
                    },
                    "criteria": routes,
                },
                "semantic_hazard": {
                    "type": "noul",
                    "instructions": (
                        "Does the current structured robot state contain semantic evidence that "
                        "continuing autonomous motion may harm a person, object, robot, or violate "
                        "the stated task or workspace constraints?"
                    ),
                    "criteria": {
                        "true": "There is evidence of a hazard or a material constraint violation.",
                        "false": "There is no semantic evidence of a hazard in the supplied state.",
                    },
                },
            },
        }
        try:
            result = self._request(payload)
            route, confidence, hazard, model = self._parse_response(result, routes)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            return self._hold("fallback", f"Jev unavailable or invalid: {exc}")

        if hazard >= self.hazard_stop_threshold or route == "stop":
            reason = (
                "semantic hazard threshold reached"
                if hazard >= self.hazard_stop_threshold
                else "Jev selected stop"
            )
            return SupervisorDecision(
                "stop", False, "jev", reason,
                confidence, hazard, model,
            )
        if route == "hold" or confidence < self.route_confidence_min:
            reason = "Jev selected hold" if route == "hold" else "route confidence below threshold"
            return SupervisorDecision("hold", False, "jev", reason, confidence, hazard, model)
        return SupervisorDecision(
            route, route not in {"complete", "stop", "hold"}, "jev", "accepted Jev route",
            confidence, hazard, model,
        )

    def _validate_inputs(self, state, routes):
        if not isinstance(state, dict):
            return self._hold("local", "state must be a JSON object")
        safety = state.get("local_safety")
        if not isinstance(safety, dict) or safety.get("safe") is not True:
            return SupervisorDecision(
                "stop", False, "local", "local safety state is missing or unsafe"
            )
        if not 2 <= len(routes) <= 255 or not {"hold", "stop"}.issubset(routes):
            return self._hold("local", "candidate routes must include hold and stop")
        if not all(isinstance(key, str) and isinstance(value, str) for key, value in routes.items()):
            return self._hold("local", "candidate routes must map strings to descriptions")
        return None

    def _request(self, payload):
        request = urllib.request.Request(
            API_URL,
            data=json.dumps(payload, ensure_ascii=False).encode(),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            raise OSError(f"HTTP {exc.code}") from exc
        except urllib.error.URLError as exc:
            raise OSError(str(exc.reason)) from exc

    @staticmethod
    def _parse_response(result, routes):
        answers = result["answers"]
        route_answer = answers["next_route"]
        hazard_answer = answers["semantic_hazard"]
        if route_answer.get("type") != "choice" or hazard_answer.get("type") != "noul":
            raise ValueError("unexpected Jev answer types")
        route = route_answer["choice"]
        confidence = float(route_answer["confidence"])
        hazard = float(hazard_answer["noul"])
        if route not in routes or not 0 <= confidence <= 1 or not 0 <= hazard <= 1:
            raise ValueError("Jev answer is outside the requested contract")
        return route, confidence, hazard, result.get("model")

    @staticmethod
    def _hold(source, reason):
        return SupervisorDecision("hold", False, source, reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state", type=Path, help="JSON state snapshot; may include candidate_routes")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--model")
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--route-confidence", type=float)
    parser.add_argument("--hazard-stop", type=float)
    args = parser.parse_args()

    state = json.loads(args.state.read_text())
    routes = state.pop("candidate_routes", DEFAULT_ROUTES) if isinstance(state, dict) else DEFAULT_ROUTES
    config = json.loads(args.config.read_text()) if args.config.exists() else {}
    if not isinstance(config, dict):
        parser.error("Jev config must be a JSON object")
    api_key = (
        os.environ.get("TYPESAFE_API_KEY")
        or config.get("api_key")
        or getpass.getpass("TypeSafe API key: ")
    )
    if not api_key:
        decision = SupervisorDecision("hold", False, "fallback", "TypeSafe API key is empty")
    else:
        try:
            decision = JevSupervisor(
                api_key,
                model=args.model or config.get("model", DEFAULT_MODEL),
                timeout_s=(
                    args.timeout
                    if args.timeout is not None
                    else float(config.get("timeout_s", 2.0))
                ),
                route_confidence_min=(
                    args.route_confidence
                    if args.route_confidence is not None
                    else float(config.get("route_confidence_min", 0.75))
                ),
                hazard_stop_threshold=(
                    args.hazard_stop
                    if args.hazard_stop is not None
                    else float(config.get("hazard_stop_threshold", 0.70))
                ),
            ).evaluate(state, routes)
        except (TypeError, ValueError) as exc:
            parser.error(str(exc))
    print(json.dumps(decision.as_dict(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
