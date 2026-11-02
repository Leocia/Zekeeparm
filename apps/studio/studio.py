"""Local-only Zekeeparm Studio service. No ROS, Node runtime or cloud required."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import signal
import threading
from urllib.parse import unquote, urlsplit
import webbrowser

from core import Session, compile_program, joints, number, MAX_OPENING
from robot import ROOT, RobotModel, READY, HARDWARE, CollisionError

PUBLIC = Path(__file__).resolve().parent / "public"


def default_data_dir():
    base = Path(os.environ.get("LOCALAPPDATA", Path.home() / ".local/share"))
    return base / "ZekeeparmStudio"


class App:
    def __init__(self, directory):
        self.model = RobotModel()
        self.session = Session(self.model)
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.files_lock = threading.Lock()

    def project_path(self, name):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80:
            raise ValueError("项目名需为 1–80 个字符")
        return self.directory / (hashlib.sha256(name.encode()).hexdigest() + ".json")

    def points(self, points):
        if not isinstance(points, dict) or len(points) > 200:
            raise ValueError("最多保存 200 个示教点")
        for name, value in points.items():
            if not isinstance(name, str) or not 1 <= len(name) <= 60:
                raise ValueError("示教点名称需为 1–60 个字符")
            joints(value, self.model.limits)
        return points

    def steps(self, body):
        return compile_program(body["program"], self.points(body["points"]), self.model.limits)

    def post(self, route, body):
        fields = {
            "/api/heartbeat": set(), "/api/connect": {"mode", "channel"},
            "/api/disconnect": set(), "/api/enable": {"enabled"}, "/api/stop": set(),
            "/api/run": {"program", "points"}, "/api/check": {"program", "points"},
            "/api/collision": {"joints", "opening"},
            "/api/fk": {"joints"}, "/api/ik": {"position_m", "rpy_rad", "seed"},
            "/api/projects/save": {"name", "workspace", "points"},
            "/api/projects/load": {"name"}, "/api/logs/clear": set(),
        }
        if route not in fields:
            raise FileNotFoundError("接口不存在")
        if not isinstance(body, dict) or set(body) != fields[route]:
            raise ValueError("请求参数缺失或包含未知字段")
        session = self.session
        if route == "/api/heartbeat":
            import time
            session.last_heartbeat = time.monotonic()
        elif route == "/api/logs/clear":
            with session.lock:
                session.events.clear()
        elif route == "/api/connect":
            session.connect(body["mode"], body["channel"])
        elif route == "/api/disconnect":
            session.disconnect()
        elif route == "/api/enable":
            if type(body["enabled"]) is not bool:
                raise ValueError("enabled 必须为布尔值")
            session.enable(body["enabled"])
        elif route == "/api/stop":
            session.stop()
        elif route == "/api/run":
            session.start(self.steps(body), self.points(body["points"]))
        elif route == "/api/check":
            with session.lock:
                session.idle()
                steps = session.prepare(self.steps(body), points=self.points(body["points"]))
                return {"steps": len(steps), "duration": sum(s.get("seconds", s.get("duration", 0.2)) for s in steps),
                        "prepared": steps, "message": "参数、自碰撞与环境碰撞检查通过；未发送运动"}
        elif route == "/api/collision":
            q = joints(body["joints"], self.model.limits)
            opening = number(body["opening"], 0, MAX_OPENING, "夹爪开口")
            try:
                self.model.check_path(q, q, opening)
            except CollisionError as exc:
                return {"collision": True, "kind": exc.kind, "links": exc.links, "message": str(exc)}
            return {"collision": False, "links": []}
        elif route == "/api/fk":
            return self.model.fk(joints(body["joints"], self.model.limits))
        elif route == "/api/ik":
            vectors = []
            for name, bound in (("position_m", 2), ("rpy_rad", 3.141592653589793)):
                value = body[name]
                if not isinstance(value, list) or len(value) != 3:
                    raise ValueError(f"{name} 需要三个数值")
                vectors.append([number(v, -bound, bound, name) for v in value])
            seed = joints(body["seed"], self.model.limits)
            q = joints(self.model.ik(*vectors, seed), self.model.limits)
            return {"joints": q, "pose": self.model.fk(q)}
        elif route == "/api/projects/save":
            self.points(body["points"])
            if not isinstance(body["workspace"], dict):
                raise ValueError("积木工作区格式无效")
            path = self.project_path(body["name"])
            with self.files_lock:
                temporary = path.with_suffix(".tmp")
                temporary.write_text(json.dumps({"format": "zekeeparm_studio_v1", **body}, ensure_ascii=False), encoding="utf-8")
                temporary.replace(path)
            return {"message": "项目已保存到本机"}
        elif route == "/api/projects/load":
            with self.files_lock:
                return json.loads(self.project_path(body["name"]).read_text(encoding="utf-8"))
        return {"ok": True}


def make_server(app, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def allowed(self):
            hosts = (f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}")
            host = self.headers.get("Host", "")
            origin = self.headers.get("Origin")
            return host in hosts and (not origin or origin == f"http://{host}")

        def send(self, status, content, mime="application/json; charset=utf-8"):
            data = json.dumps(content, ensure_ascii=False, allow_nan=False).encode() if mime.startswith("application/json") else content
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if not self.allowed():
                return self.send(403, {"error": "仅允许本机同源访问"})
            route = unquote(urlsplit(self.path).path)
            try:
                if route == "/api/state":
                    return self.send(200, app.session.state())
                if route == "/api/config":
                    return self.send(200, {"version": "0.1.0", "limits": app.model.limits,
                                           "max_opening_mm": MAX_OPENING, "tcp": app.model.frame,
                                           "collision": "self+fixed_worktable", "ros_required": False})
                if route == "/api/projects":
                    with app.files_lock:
                        projects = [json.loads(p.read_text(encoding="utf-8"))["name"] for p in app.directory.glob("*.json")]
                    return self.send(200, {"projects": sorted(projects)})
                roots = {"/model/description/": ROOT / "src/zekeep_bringup/description",
                         "/lib/": ROOT / "src/zekeep_bringup/web/public/lib"}
                base, relative = PUBLIC, "index.html" if route == "/" else route.lstrip("/")
                for prefix, directory in roots.items():
                    if route.startswith(prefix):
                        base, relative = directory, route[len(prefix):]
                path = (base / relative).resolve()
                if not path.is_relative_to(base.resolve()) or not path.is_file():
                    raise FileNotFoundError("文件不存在")
                mime = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
                self.send(200, path.read_bytes(), mime)
            except (OSError, ValueError, RuntimeError) as exc:
                self.send(404 if isinstance(exc, FileNotFoundError) else 400, {"error": str(exc)})

        def do_POST(self):
            if not self.allowed() or self.headers.get("X-Studio-Request") != "1":
                return self.send(403, {"error": "仅允许本机应用请求"})
            try:
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise ValueError("需要 application/json")
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024 * 1024:
                    raise ValueError("请求大小须在 1 B–1 MiB 之间")
                self.connection.settimeout(15)
                body = json.loads(self.rfile.read(size))
                self.send(200, app.post(urlsplit(self.path).path, body))
            except Exception as exc:
                status = 404 if isinstance(exc, FileNotFoundError) else 409 if isinstance(exc, RuntimeError) else 400
                self.send(status, {"error": str(exc), "block_id": getattr(exc, "block_id", ""),
                                   "collision_links": getattr(exc, "links", [])})

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--data-dir", type=Path, default=default_data_dir())
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--self-test", action="store_true", help="Check bundled imports, model and assets without hardware")
    args = parser.parse_args()
    if not (PUBLIC / "vendor/blockly/blockly_compressed.js").is_file():
        parser.error("缺少 Blockly 资源。请在 apps/studio 中执行 npm ci")
    if args.self_test:
        from zekeepcontroller.hardware_manager import HardwareManager
        hardware = HardwareManager(str(HARDWARE), channel="COM3", auto_enable=False)
        if hardware.enabled:
            raise RuntimeError("Hardware constructor unexpectedly enabled outputs")
        model = RobotModel()
        pose = model.fk(READY)
        model.ik(pose["position_m"], pose["rpy_rad"], READY)
        model.check_path(READY, READY, 40)
        if not (ROOT / "src/zekeep_bringup/web/public/lib/URDFLoader.js").is_file():
            raise FileNotFoundError("URDF loader missing from bundle")
        from python_block import run_code
        calculated, _, _ = run_code("result = [q for q in joints]", {}, READY)
        if calculated != READY:
            raise RuntimeError("Python calculation self-test failed")
        print("Studio model, FK/IK, Python block, collision geometry, SDK imports and offline assets: OK")
        return
    app = App(args.data_dir)
    server = make_server(app, args.port)

    def interrupt(_signal, _frame):
        app.session.stop()
        if app.session.device and app.session.device.manager.enabled:
            print("机械臂仍保持使能。请在界面支撑并失能后，再关闭服务。", flush=True)
            return
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, interrupt)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, interrupt)
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"Zekeeparm Studio: {url}\n数据目录: {app.directory}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        if app.session.connected:
            app.session.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
