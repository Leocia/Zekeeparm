"""Bounded program validation and a single cancelable local executor."""
from collections import deque
from contextlib import contextmanager
import copy
import math
import json
import operator
import threading
import time

from robot import READY

MAX_OPENING = 70 * 1.35 / 1.45
MAX_STEPS = 2000


class ProgramError(ValueError):
    def __init__(self, message, block_id=""):
        super().__init__(message)
        self.block_id = block_id


@contextmanager
def block_context(item):
    try:
        yield
    except (ProgramError, InterruptedError):
        raise
    except (ValueError, TypeError, KeyError, ArithmeticError) as exc:
        raise ProgramError(str(exc), item.get("id", "") if isinstance(item, dict) else "") from exc


def number(value, low, high, label):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} 必须在 {low:g}–{high:g} 范围内")
    return float(value)


def joints(value, limits):
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError("需要六个关节角，单位 rad")
    return [number(v, lo, hi, f"J{i + 1}") for i, (v, (lo, hi)) in enumerate(zip(value, limits))]


def name(value):
    if not isinstance(value, str) or not 1 <= len(value.strip()) <= 60:
        raise ValueError("名称需为 1–60 个字符")
    return value


def profile(value):
    if value not in ("cubic", "min_jerk", "trapezoid", "linear"):
        raise ValueError("未知轨迹曲线")
    return value


LOGIC_FIELDS = {
    "literal": {"value"}, "variable": {"name"}, "arithmetic": {"op", "a", "b"},
    "unary": {"op", "value"}, "compare": {"op", "a", "b"},
    "logic": {"op", "a", "b"}, "not": {"value"}, "list": {"items"},
    "list_get": {"list", "index"}, "length": {"value"}, "dict_get": {"dict", "key"},
}
ARITHMETIC = {"add": operator.add, "sub": operator.sub, "mul": operator.mul,
              "div": operator.truediv, "mod": operator.mod}
COMPARISON = {"eq": operator.eq, "ne": operator.ne, "lt": operator.lt,
              "le": operator.le, "gt": operator.gt, "ge": operator.ge}
UNARY = {"round": round, "floor": math.floor, "ceil": math.ceil, "abs": abs,
         "neg": operator.neg, "to_rad": math.radians, "to_deg": math.degrees}


def scalar(value):
    return number(value, -1e12, 1e12, "数值")


def boolean(value):
    if type(value) is not bool:
        raise ValueError("逻辑条件必须是布尔值")
    return value


def index(value, items):
    if type(value) not in (int, float) or not math.isfinite(value) or int(value) != value or not 1 <= value <= len(items):
        raise ValueError("列表序号必须是范围内的整数，从 1 开始")
    return int(value) - 1


def checked_value(value):
    nodes = 0
    def check(item, depth=0):
        nonlocal nodes
        nodes += 1
        if depth > 8 or nodes > 512:
            raise ValueError("变量值过深或过大")
        if type(item) in (int, float):
            scalar(item)
        elif type(item) in (str, bool):
            if isinstance(item, str) and len(item) > 4096:
                raise ValueError("文本最多 4096 个字符")
        elif isinstance(item, (list, tuple, dict)):
            if len(item) > 256:
                raise ValueError("列表或字典最多 256 项")
            for child in item.values() if isinstance(item, dict) else item:
                check(child, depth + 1)
        else:
            raise ValueError("不支持的变量值类型")
    check(value)
    encoded = json.dumps(value, ensure_ascii=False, allow_nan=False)
    if len(encoded) > 32768:
        raise ValueError("变量值过大")
    return json.loads(encoded)


def expression(value, limits, depth=0):
    if depth > 8 or not isinstance(value, dict):
        raise ValueError("值积木过深或未连接")
    fields = {"current_joints": set(), "joint_values": {"joints"},
              "fk": {"joints", "frame", "output"}, "ik": {"pose", "max_retries"},
              "reachable": {"pose", "max_retries"}, "model_info": set(),
              "traj_stats": {"target", "duration", "profile", "dt", "accel_ratio"}, **LOGIC_FIELDS}
    kind = value.get("type")
    if not isinstance(kind, str) or kind not in fields or set(value) != fields[kind] | {"type"}:
        raise ValueError("未知值积木或参数字段")
    result = dict(value)
    if kind in LOGIC_FIELDS:
        if kind == "literal":
            literal = value["value"]
            if type(literal) not in (int, float, str, bool):
                raise ValueError("常量必须是数字、文本或布尔值")
            result["value"] = checked_value(literal)
        elif kind == "variable":
            name(value["name"])
        else:
            options = {"arithmetic": ARITHMETIC, "compare": COMPARISON, "unary": UNARY,
                       "logic": ("and", "or")}
            if kind in options and (not isinstance(value["op"], str) or value["op"] not in options[kind]):
                raise ValueError("未知运算符")
            if kind == "list":
                if not isinstance(value["items"], list) or len(value["items"]) > 32:
                    raise ValueError("创建列表最多 32 项")
                result["items"] = [expression(v, limits, depth+1) for v in value["items"]]
            else:
                for field in LOGIC_FIELDS[kind] - {"op", "key"}:
                    result[field] = expression(value[field], limits, depth+1)
                if kind == "dict_get":
                    name(value["key"])
    elif kind == "joint_values":
        result["joints"] = joints(value["joints"], limits)
    elif kind == "fk":
        result["joints"] = expression(value["joints"], limits, depth + 1)
        if not isinstance(value["frame"], str) or len(value["frame"]) > 100:
            raise ValueError("坐标系名称无效")
        if value["output"] not in ("position_m", "rotation", "transform", "pose"):
            raise ValueError("未知 FK 输出")
    elif kind in ("ik", "reachable"):
        pose = value["pose"]
        if not isinstance(pose, list) or len(pose) != 6:
            raise ValueError("位姿需要 X/Y/Z/R/P/Y 六个数值，单位 m/rad")
        result["pose"] = [number(v, -bound, bound, "位姿") for v, bound in zip(pose, [2]*3 + [math.pi]*3)]
        retries = value["max_retries"]
        if type(retries) is not int or not 0 <= retries <= 8:
            raise ValueError("随机重试次数需为 0–8 的整数")
    elif kind == "traj_stats":
        result["target"] = expression(value["target"], limits, depth + 1)
        result["duration"] = number(value["duration"], .2, 60, "轨迹时长")
        result["dt"] = number(value["dt"], .02, 1, "采样周期")
        if result["duration"] / result["dt"] > 500:
            raise ValueError("轨迹统计最多 501 个采样点")
        result["accel_ratio"] = number(value["accel_ratio"], .01, .49, "加速占比")
        if profile(value["profile"]) == "cubic":
            raise ValueError("轨迹统计请选择最小冲击、梯形或线性曲线")
    return result


def compile_program(program, points, limits):
    work, count = 0, 0
    functions, called = {}, set()
    known_points = set(points)
    fields = {"move": {"joints", "duration"}, "point": {"name", "duration"},
              "gripper": {"opening"}, "wait": {"seconds"}, "repeat": {"count", "body"},
              "delta": {"joint", "delta", "duration"}, "save_point": {"name"},
              "home": {"max_vel", "settle_thresh", "timeout"},
              "profile": {"profile", "dt", "accel_ratio"},
              "lifecycle": {"action", "mode"}, "disable": set(),
              "show": {"value"}, "move_value": {"value", "duration"},
              "if": {"condition", "then", "else"}, "set": {"name", "value"},
              "change": {"name", "value"}, "list_set": {"name", "index", "value"},
              "call": {"name"}, "function": {"name", "body"}, "python": {"name", "code"}}
    commands = []
    for item in program:
        with block_context(item):
            if isinstance(item, dict) and item.get("type") == "function":
                if set(item) != fields["function"] | {"id", "type"} or not isinstance(item["id"], str) or len(item["id"]) > 128:
                    raise ValueError("函数定义字段无效")
                key = name(item["name"])
                if key in functions or len(functions) >= 32 or not isinstance(item["body"], list):
                    raise ValueError("函数重名、超过 32 个或函数体无效")
                functions[key] = item["body"]
            else:
                commands.append(item)

    def visit(items, depth=0, stack=()):
        nonlocal work, count
        if not isinstance(items, list) or depth > 8 or len(items) > MAX_STEPS:
            raise ValueError("程序过深、过长或动作列表无效")
        work += len(items) + 1
        if work > MAX_STEPS * 4:
            raise ValueError("循环展开工作量过大")
        result = []
        for item in items:
            with block_context(item):
                if not isinstance(item, dict) or not isinstance(item.get("id"), str) or len(item["id"]) > 128:
                    raise ValueError("积木缺少有效 ID")
                kind = item.get("type")
                if not isinstance(kind, str) or kind not in fields or set(item) != fields[kind] | {"id", "type"}:
                    raise ValueError("未知积木或参数字段")
                step = dict(item)
                if kind == "repeat":
                    repeat = item["count"]
                    if type(repeat) is not int or not 1 <= repeat <= 100:
                        raise ValueError("循环次数需为 1–100 的整数")
                    for _ in range(repeat):
                        result.extend(visit(item["body"], depth + 1, stack))
                    continue
                count += 1
                if count > MAX_STEPS:
                    raise ValueError(f"展开后最多 {MAX_STEPS} 个动作")
                if kind == "call":
                    key = name(item["name"])
                    if key not in functions:
                        raise ValueError(f"函数不存在：{key}")
                    if key in stack:
                        raise ValueError("函数不支持递归调用")
                    called.add(key)
                    result.extend(visit(functions[key], depth+1, stack+(key,)))
                    continue
                if kind == "function":
                    raise ValueError("函数定义必须独立放在工作区顶层")
                if "duration" in fields[kind]:
                    step["duration"] = number(item["duration"], .2, 60, "运动时长")
                if kind == "move":
                    step["joints"] = joints(item["joints"], limits)
                elif kind == "point":
                    if name(item["name"]) not in known_points:
                        raise ValueError("示教点不存在，请重新选择")
                    if item["name"] in points:
                        step = dict(id=item["id"], type="move", joints=joints(points[item["name"]], limits), duration=step["duration"])
                elif kind == "wait":
                    step["seconds"] = number(item["seconds"], 0, 60, "等待秒数")
                elif kind == "gripper":
                    step["opening"] = number(item["opening"], 0, MAX_OPENING, "夹爪开口 mm")
                elif kind == "delta":
                    if type(item["joint"]) is not int or not 0 <= item["joint"] < 6:
                        raise ValueError("关节应为 J1–J6")
                    step["delta"] = number(item["delta"], -math.pi, math.pi, "相对角度 rad")
                elif kind == "save_point":
                    known_points.add(name(item["name"]))
                elif kind == "home":
                    step["max_vel"] = number(item["max_vel"], .01, .3, "回零角速度 rad/s")
                    step["settle_thresh"] = number(item["settle_thresh"], .001, .05, "收敛阈值 rad")
                    step["timeout"] = number(item["timeout"], 1, 120, "到位超时 s")
                elif kind == "profile":
                    profile(item["profile"])
                    step["dt"] = number(item["dt"], .02, .1, "采样周期 s")
                    step["accel_ratio"] = number(item["accel_ratio"], .01, .49, "加速占比")
                elif kind == "lifecycle":
                    if item["action"] not in ("start", "end") or item["mode"] not in ("posvel", "mit"):
                        raise ValueError("控制器启停参数无效")
                elif kind == "python":
                    from python_block import validate_code
                    name(item["name"])
                    validate_code(item["code"])
                elif kind in ("set", "change", "list_set"):
                    name(item["name"])
                    step["value"] = expression(item["value"], limits)
                    if kind == "list_set":
                        step["index"] = expression(item["index"], limits)
                elif kind in ("show", "move_value"):
                    step["value"] = expression(item["value"], limits)
                elif kind == "if":
                    step["condition"] = expression(item["condition"], limits)
                    before = set(known_points)
                    step["then"] = visit(item["then"], depth + 1, stack)
                    after_then = set(known_points)
                    known_points.clear(); known_points.update(before)
                    step["else"] = visit(item["else"], depth + 1, stack)
                    known_points.intersection_update(after_then)
                result.append(step)
        return result

    result = visit(commands)
    for key, body in functions.items():
        if key not in called:
            visit(body, stack=(key,))
    if not result:
        raise ValueError("程序为空，请连接动作积木")
    def cost(value):
        kind = value["type"]
        if kind in ("ik", "reachable"):
            return 500 * (value["max_retries"] + 1)
        if kind == "traj_stats":
            return (math.ceil(value["duration"] / value["dt"]) + 1) * 200 + cost(value["target"])
        children = [v for v in value.values() if isinstance(v, dict)]
        if kind == "list":
            children.extend(value["items"])
        return 1 + sum(cost(v) for v in children)
    def calculation_cost(items):
        return sum(cost(s["condition"]) + calculation_cost(s["then"]) + calculation_cost(s["else"])
                   if s["type"] == "if" else 10000 if s["type"] == "python" else sum(cost(s[k]) for k in ("value", "index") if k in s) for s in items)
    if calculation_cost(result) > 150000:
        raise ValueError("运动学计算量过大，请减少采样、重试或循环次数")
    def budget(items):
        return sum(max(budget(s["then"]), budget(s["else"])) if s["type"] == "if"
                   else s.get("seconds", s.get("duration", 3)) for s in items)
    if budget(result) > 1800:
        raise ValueError("程序时长超过 30 分钟，请拆分任务")
    return result


def sample(t, curve="cubic", accel=.25):
    if curve == "min_jerk":
        return 10*t**3 - 15*t**4 + 6*t**5, 30*t*t*(1-t)**2
    if curve == "linear":
        return t, 1
    if curve == "trapezoid":
        speed = 1 / (1-accel)
        if t < accel:
            return speed*t*t/(2*accel), speed*t/accel
        if t <= 1-accel:
            return speed*(t-accel/2), speed
        return 1-speed*(1-t)**2/(2*accel), speed*(1-t)/accel
    return t*t*(3-2*t), 6*t*(1-t)


class Session:
    def __init__(self, model):
        self.model = model
        self.lock = threading.RLock()
        self.command_lock = threading.RLock()
        self.device = None
        self.connected = False
        self.mode = "simulation"
        self.enabled = False
        self.q = list(READY)
        self.opening = 40.0
        self.cancelled = threading.Event()
        self.worker = None
        self.phase = "idle"
        self.current = ""
        self.completed = 0
        self.total = 0
        self.error = ""
        self.error_links = []
        self.error_block = ""
        self.events = deque(maxlen=80)
        self.last_heartbeat = time.monotonic()
        self.blocked = False
        self.points = {}
        self.point_revision = 0
        self.control_mode = "posvel"

    @property
    def busy(self):
        return bool(self.worker and self.worker.is_alive())

    def log(self, message):
        self.events.append({"time": time.strftime("%H:%M:%S"), "message": message})

    def idle(self):
        if self.busy or self.blocked:
            raise RuntimeError("任务运行中或停止未确认，当前操作不可用")

    def state(self):
        with self.lock:
            if self.device:
                state = self.device.state()
                self.q, self.opening, self.enabled = state["joints"], state["opening_mm"], state["enabled"]
            return {"connected": self.connected, "mode": self.mode, "enabled": self.enabled,
                    "joints": list(self.q), "opening_mm": self.opening, "busy": self.busy,
                    "phase": self.phase, "current": self.current, "completed": self.completed,
                    "total": self.total, "error": self.error, "error_block": self.error_block,
                    "error_links": list(self.error_links), "blocked": self.blocked,
                    "events": list(self.events), "points": copy.deepcopy(self.points),
                    "control_mode": self.control_mode, "point_revision": self.point_revision}

    def connect(self, mode, channel=""):
        with self.lock:
            self.idle()
            if self.connected:
                raise RuntimeError("请先断开当前设备")
            if mode not in ("simulation", "sdk"):
                raise ValueError("未知连接模式")
            if mode == "sdk":
                import re
                if not isinstance(channel, str) or not re.fullmatch(r"COM[1-9][0-9]*|/dev/tty(?:ACM|USB)[0-9]+", channel, re.I):
                    raise ValueError("串口应为 COM3 或 /dev/ttyACM0")
                from robot import SDKDevice
                self.device = SDKDevice(channel, self.model)
            self.mode, self.connected, self.enabled = mode, True, False
            self.error = ""
            self.log("已连接模拟设备" if mode == "simulation" else f"已连接真机 {channel}，尚未使能")
            return self.state()

    def disconnect(self):
        with self.lock:
            self.idle()
            if self.device:
                self.device.close()
            self.device = None
            self.connected = self.enabled = False
            self.log("设备已断开")

    def enable(self, value):
        with self.lock:
            self.idle()
            if not self.connected:
                raise RuntimeError("请先连接设备")
            if self.device:
                self.device.enable() if value else self.device.disable()
            self.enabled = value
            self.log("设备已使能" if value else "设备已失能")

    def evaluate(self, value, q, checkpoint, variables=None):
        checkpoint()
        kind = value["type"]
        variables = {} if variables is None else variables
        child = lambda item: self.evaluate(item, q, checkpoint, variables)
        if kind in LOGIC_FIELDS:
            if kind == "literal":
                return value["value"]
            if kind == "variable":
                if value["name"] not in variables:
                    raise ValueError(f"变量未赋值：{value['name']}")
                return copy.deepcopy(variables[value["name"]])
            if kind == "arithmetic":
                a, b = scalar(child(value["a"])), scalar(child(value["b"]))
                if value["op"] in ("div", "mod") and b == 0:
                    raise ValueError("除数不能为 0")
                return scalar(ARITHMETIC[value["op"]](a, b))
            if kind == "unary":
                return scalar(UNARY[value["op"]](scalar(child(value["value"]))))
            if kind == "compare":
                a, b = child(value["a"]), child(value["b"])
                if value["op"] not in ("eq", "ne"):
                    a, b = scalar(a), scalar(b)
                elif type(a) is not type(b) and not (type(a) in (int, float) and type(b) in (int, float)):
                    return value["op"] == "ne"
                return COMPARISON[value["op"]](a, b)
            if kind == "logic":
                first = boolean(child(value["a"]))
                return (first and boolean(child(value["b"]))) if value["op"] == "and" else (first or boolean(child(value["b"])))
            if kind == "not":
                return not boolean(child(value["value"]))
            if kind == "list":
                return checked_value([child(v) for v in value["items"]])
            if kind == "list_get":
                items = child(value["list"])
                if not isinstance(items, list):
                    raise ValueError("取列表项需要列表")
                return copy.deepcopy(items[index(child(value["index"]), items)])
            if kind == "length":
                items = child(value["value"])
                if not isinstance(items, (list, dict, str)):
                    raise ValueError("长度只接受列表、字典或文本")
                return len(items)
            items = child(value["dict"])
            if not isinstance(items, dict) or value["key"] not in items:
                raise ValueError(f"字典不存在字段：{value['key']}")
            return copy.deepcopy(items[value["key"]])
        if kind == "current_joints":
            return list(q)
        if kind == "joint_values":
            return value["joints"]
        if kind == "model_info":
            return self.model.info()
        if kind == "fk":
            target = joints(child(value["joints"]), self.model.limits)
            pose = self.model.fk(target, value["frame"])
            return pose if value["output"] == "pose" else pose[value["output"]]
        if kind in ("ik", "reachable"):
            try:
                target = self.model.ik(value["pose"][:3], value["pose"][3:], q, value["max_retries"], checkpoint)
                target = joints(target, self.model.limits)
            except ValueError:
                if kind == "reachable":
                    return False
                raise
            return True if kind == "reachable" else target
        target = joints(child(value["target"]), self.model.limits)
        return self.model.trajectory_stats(q, target, value["duration"], value["profile"],
                                           value["dt"], value["accel_ratio"], checkpoint)

    def prepare(self, steps, cancelable=False, points=None):
        previous = self.state()["joints"]
        opening = self.opening
        self.model.check_path(previous, previous, opening)
        points = copy.deepcopy(self.points if points is None else points)
        prepared = []
        variables = {}
        curve, dt, accel = "cubic", .02, .25
        enabled = self.enabled if cancelable else True
        checkpoint = self.checkpoint if cancelable else lambda: None

        def visit(items):
            nonlocal previous, opening, curve, dt, accel, enabled
            for original in items:
                with block_context(original):
                    checkpoint()
                    step = dict(original)
                    kind = step["type"]
                    if kind == "if":
                        result = boolean(self.evaluate(step["condition"], previous, checkpoint, variables))
                        visit(step["then"] if result else step["else"])
                        continue
                    if kind == "python":
                        from python_block import run_code
                        result, updated, output = run_code(step["code"], variables, previous, checkpoint)
                        updated[step["name"]] = checked_value(result)
                        if len(updated) > 64:
                            raise ValueError("最多 64 个变量")
                        variables.clear(); variables.update(updated)
                        step = dict(id=step["id"], type="show", result={"Python": step["name"], "result": result, "print": output})
                    elif kind in ("set", "change", "list_set"):
                        key = step["name"]
                        result = self.evaluate(step["value"], previous, checkpoint, variables)
                        if kind != "set" and key not in variables:
                            raise ValueError(f"变量未赋值：{key}")
                        if kind == "change":
                            result = scalar(scalar(variables[key]) + scalar(result))
                        elif kind == "list_set":
                            items = copy.deepcopy(variables[key])
                            if not isinstance(items, list):
                                raise ValueError("修改列表项需要列表变量")
                            position = self.evaluate(step["index"], previous, checkpoint, variables)
                            items[index(position, items)] = result
                            result = items
                        if key not in variables and len(variables) >= 64:
                            raise ValueError("最多 64 个变量")
                        variables[key] = checked_value(result)
                        step = dict(id=step["id"], type="show", result={"variable": key, "value": variables[key]})
                    elif kind == "profile":
                        curve, dt, accel = step["profile"], step["dt"], step["accel_ratio"]
                        step = dict(id=step["id"], type="show", result={"profile": curve, "dt": dt, "accel_ratio": accel})
                    elif kind == "disable":
                        enabled = False
                    elif kind == "lifecycle":
                        if self.device:
                            self.device.validate_lifecycle(step["mode"])
                        if step["action"] == "end":
                            if not enabled:
                                raise ValueError("安全结束前请启动位置控制")
                            visit([dict(id=step["id"], type="home", max_vel=.15, settle_thresh=.01, timeout=30)])
                        enabled = step["action"] == "start"
                    elif kind == "show":
                        step["result"] = self.evaluate(step["value"], previous, checkpoint, variables)
                        def uses_variable(item):
                            if isinstance(item, dict):
                                return item.get("type") == "variable" or any(uses_variable(v) for v in item.values())
                            return isinstance(item, list) and any(uses_variable(v) for v in item)
                        if uses_variable(step["value"]):
                            del step["value"]
                    elif kind == "save_point":
                        if step["name"] in points:
                            raise ValueError("示教点名称已存在，请使用其他名称")
                        if len(points) >= 200:
                            raise ValueError("最多保存 200 个示教点")
                        points[step["name"]] = list(previous)
                    elif kind in ("delta", "home", "move_value", "point"):
                        if kind == "delta":
                            target = list(previous)
                            i = step["joint"]
                            lo, hi = self.model.limits[i]
                            target[i] = max(lo, min(hi, target[i] + step["delta"]))
                        elif kind == "home":
                            target = self.model.home
                        elif kind == "point":
                            if step["name"] not in points:
                                raise ValueError("示教点不存在")
                            target = points[step["name"]]
                        else:
                            target = joints(self.evaluate(step["value"], previous, checkpoint, variables), self.model.limits)
                        converted = dict(id=step["id"], type="move", joints=joints(list(target), self.model.limits), duration=step.get("duration", .2))
                        if kind == "home":
                            converted.update(settle_thresh=step["settle_thresh"], timeout=step["timeout"], max_vel=step["max_vel"])
                        if kind == "delta":
                            converted.update(relative_joint=step["joint"], relative_delta=step["delta"])
                        elif kind == "point":
                            converted["point_name"] = step["name"]
                        step = converted
                        if kind == "home":
                            step["profile"] = "min_jerk"
                    if step["type"] in ("move", "gripper"):
                        if not enabled:
                            raise ValueError("运动前必须启动位置控制")
                    if step["type"] == "move":
                        selected = step.get("profile", curve)
                        peak = {"cubic": 1.5, "min_jerk": 1.875, "linear": 1, "trapezoid": 1/(1-accel)}[selected]
                        step["duration"] = max(step["duration"], max(abs(a-b) for a,b in zip(previous, step["joints"])) * peak / step.get("max_vel", .30))
                        step.update(profile=selected, dt=dt, accel_ratio=accel)
                        self.model.check_path(previous, step["joints"], opening)
                        previous = step["joints"]
                    elif step["type"] == "gripper":
                        for i in range(11):
                            checkpoint()
                            self.model.check_path(previous, previous, opening + (step["opening"]-opening)*i/10)
                        opening = step["opening"]
                    prepared.append(step)
        visit(steps)
        if sum(s.get("seconds", s.get("duration", 3)) + s.get("timeout", 0) for s in prepared) > 1800:
            raise ValueError("调整速度后程序超过 30 分钟")
        return prepared

    def start(self, steps, points=None):
        with self.lock:
            self.idle()
            if not self.connected:
                raise RuntimeError("请先连接设备")
            steps = copy.deepcopy(steps)
            self.points = {}
            self.cancelled.clear()
            self.phase, self.error, self.current = "checking", "", ""
            self.error_links = []
            self.error_block = ""
            self.completed, self.total = 0, len(steps)
            self.last_heartbeat = time.monotonic()
            self.worker = threading.Thread(target=self.run, args=(steps, copy.deepcopy(points)), daemon=True)
            self.worker.start()

    def checkpoint(self):
        if time.monotonic() - self.last_heartbeat > 5:
            self.cancelled.set()
        if self.cancelled.is_set():
            raise InterruptedError("任务已停止")

    def run(self, steps, points=None):
        try:
            initial = self.state()["joints"]
            prepared = self.prepare(steps, cancelable=True, points=points)
            self.total = len(prepared)
            self.checkpoint()
            if self.device and max(abs(a-b) for a,b in zip(self.state()["joints"], initial)) > 0.01:
                raise RuntimeError("检查期间姿态发生变化，请重新运行")
            self.phase = "running"
            self.log(f"开始运行，共 {len(prepared)} 个动作")
            for step in prepared:
                self.checkpoint()
                self.current = step["id"]
                if step["type"] == "move":
                    self.move(step)
                elif step["type"] == "wait":
                    self.wait(step["seconds"])
                elif step["type"] == "show":
                    result = self.evaluate(step["value"], self.state()["joints"], self.checkpoint) if "value" in step else step["result"]
                    self.log(json.dumps(result, ensure_ascii=False))
                elif step["type"] == "save_point":
                    q = self.state()["joints"]
                    pose = self.model.fk(q)
                    with self.lock:
                        self.points[step["name"]] = list(q)
                        self.point_revision += 1
                    self.log(f"已记录示教点 {step['name']}：" + json.dumps(pose, ensure_ascii=False))
                elif step["type"] == "disable":
                    with self.command_lock:
                        self.checkpoint()
                        if self.device:
                            self.device.disable()
                        self.enabled = False
                    self.log("设备已失能，电机力矩已释放")
                elif step["type"] == "lifecycle":
                    with self.command_lock:
                        self.checkpoint()
                        if step["action"] == "end":
                            # Keep torque enabled after homing; power release stays explicit.
                            if self.device:
                                self.device.stop()
                        else:
                            if self.device:
                                self.device.lifecycle(step["mode"])
                            self.enabled = True
                            self.control_mode = step["mode"]
                    self.log("控制器已启动" if step["action"] == "start" else "已回零并保持，失能需显式操作")
                elif step["type"] == "gripper":
                    if self.device:
                        with self.command_lock:
                            self.checkpoint()
                            self.device.gripper(step["opening"])
                        deadline = time.monotonic() + 5
                        while abs(self.state()["opening_mm"] - step["opening"]) > 3:
                            self.wait(0.03)
                            if time.monotonic() > deadline:
                                raise RuntimeError("夹爪未到达目标开口")
                    else:
                        with self.lock:
                            self.opening = step["opening"]
                        self.wait(0.2)
                self.checkpoint()
                self.completed += 1
            with self.command_lock:
                self.checkpoint()
                self.phase = "completed"
            self.log("程序运行完成")
        except InterruptedError:
            self.phase = "stopped"
            self.log("任务已停止")
        except Exception as exc:
            self.phase, self.error = "error", str(exc)
            self.error_links = getattr(exc, "links", [])
            self.error_block = getattr(exc, "block_id", self.current)
            self.log(str(exc))
        finally:
            if self.device and self.phase != "completed":
                try:
                    self.hold()
                except RuntimeError:
                    self.phase = "error"
                    self.log(self.error)

    def wait(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.checkpoint()
            self.cancelled.wait(min(0.03, max(0, deadline-time.monotonic())))

    def move(self, step):
        start = self.state()["joints"]
        target, duration = list(step["joints"]), step["duration"]
        if "relative_joint" in step:
            i = step["relative_joint"]
            lo, hi = self.model.limits[i]
            target = list(start)
            target[i] = max(lo, min(hi, start[i] + step["relative_delta"]))
        elif "point_name" in step and step["point_name"] in self.points:
            target = list(self.points[step["point_name"]])
        curve, accel = step.get("profile", "cubic"), step.get("accel_ratio", .25)
        peak = {"cubic": 1.5, "min_jerk": 1.875, "linear": 1, "trapezoid": 1/(1-accel)}[curve]
        duration = max(duration, max(abs(a-b) for a,b in zip(start, target))*peak/step.get("max_vel", .30))
        if self.device:
            self.model.check_path(start, target, self.opening)
            with self.command_lock:
                self.checkpoint()
                self.device.begin()
        began = time.monotonic()
        try:
            while True:
                self.checkpoint()
                t = min(1, (time.monotonic() - began) / duration)
                blend, speed = sample(t, step.get("profile", "cubic"), step.get("accel_ratio", .25))
                q = [a+(b-a)*blend for a,b in zip(start,target)]
                velocity = [(b-a)*(speed if t < 1 else 0)/duration for a,b in zip(start,target)]
                if self.device:
                    with self.command_lock:
                        self.checkpoint()
                        self.device.target(q, velocity)
                    actual = self.state()["joints"]
                    if max(abs(a-b) for a,b in zip(q,actual)) > 0.35:
                        raise RuntimeError("实际关节偏离轨迹，已请求停止")
                else:
                    with self.lock:
                        self.q = q
                if t >= 1:
                    break
                self.wait(step.get("dt", .02))
            if self.device:
                deadline, settled = time.monotonic() + step.get("timeout", 5), 0
                while settled < 3:
                    actual = self.device.state()
                    settled = settled + 1 if max(abs(a-b) for a,b in zip(target,actual["joints"])) < step.get("settle_thresh", .02) and max(abs(v) for v in actual["velocity"]) < 0.05 else 0
                    if time.monotonic() > deadline:
                        raise RuntimeError("运动未在规定时间内到位")
                    self.wait(0.03)
        finally:
            if self.device and not self.cancelled.is_set() and self.phase == "running":
                self.device.end()

    def hold(self):
        with self.command_lock:
            try:
                self.device.stop()
            except Exception as exc:
                self.blocked, self.error = True, f"停止保持未确认：{exc}"
                raise RuntimeError(self.error) from exc
            self.blocked = False
            if self.error.startswith("停止保持未确认"):
                self.error = ""

    def stop(self):
        self.cancelled.set()
        if self.device:
            self.hold()
        if self.worker:
            self.worker.join(timeout=8)
        if self.busy:
            raise RuntimeError("已请求停止，等待当前检查结束")
        if self.blocked:
            raise RuntimeError(self.error)
