"""Interpret a bounded Python calculation subset; never execute host Python code."""
import ast
import math
import operator
import time

from core import checked_value, name

MAX_CODE = 8192
MATH_FUNCTIONS = {n: getattr(math, n) for n in (
    'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'sqrt', 'floor', 'ceil',
    'fabs', 'exp', 'log', 'log10', 'degrees', 'radians', 'hypot', 'isfinite')}
MATH_CONSTANTS = {n: getattr(math, n) for n in ('pi', 'e', 'tau')}
FUNCTIONS = {n: f for n, f in (
    ('abs', abs), ('round', round), ('min', min), ('max', max), ('sum', sum),
    ('len', len), ('int', int), ('float', float), ('bool', bool), ('str', str),
    ('list', list), ('sorted', sorted))}
BINARY = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
          ast.Div: operator.truediv, ast.FloorDiv: operator.floordiv,
          ast.Mod: operator.mod, ast.Pow: operator.pow}
COMPARE = {ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt,
           ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge,
           ast.In: lambda a, b: a in b, ast.NotIn: lambda a, b: a not in b}
ALLOWED = (ast.Module, ast.Assign, ast.AugAssign, ast.Expr, ast.If, ast.For,
           ast.Pass, ast.Import, ast.alias, ast.Name, ast.Load, ast.Store,
           ast.Constant, ast.List, ast.Tuple, ast.Dict, ast.Subscript, ast.Slice,
           ast.BinOp, ast.UnaryOp, ast.UAdd, ast.USub, ast.Not, ast.BoolOp,
           ast.And, ast.Or, ast.Compare, ast.IfExp, ast.Call, ast.Attribute,
           ast.ListComp, ast.comprehension, *BINARY, *COMPARE)
RESERVED = set(FUNCTIONS) | {'range', 'enumerate', 'zip', 'print', 'math', 'variables', 'joints'}


def validate_code(source):
    if not isinstance(source, str) or not source.strip() or len(source) > MAX_CODE:
        raise ValueError('Python 代码需为 1–8192 个字符')
    try:
        tree = ast.parse(source, mode='exec')
    except (SyntaxError, ValueError, RecursionError) as exc:
        raise ValueError(f'Python 语法错误，第 {getattr(exc, "lineno", 1)} 行：{getattr(exc, "msg", str(exc))}') from exc
    def visit(node, depth=0):
        if depth > 32 or not isinstance(node, ALLOWED):
            raise ValueError(f'Python 第 {getattr(node, "lineno", 1)} 行不支持此语法：{type(node).__name__}')
        if isinstance(node, ast.Name) and node.id.startswith('_'):
            raise ValueError('Python 不允许以下划线开头的名称')
        if isinstance(node, ast.Import) and (len(node.names) != 1 or node.names[0].name != 'math' or node.names[0].asname):
            raise ValueError('Python 只支持 import math，不支持其他模块或别名')
        if isinstance(node, ast.Attribute) and (node.attr.startswith('_') or node.attr not in set(MATH_FUNCTIONS) | set(MATH_CONSTANTS) | {'append', 'get'}):
            raise ValueError('Python 不支持此属性或方法')
        if isinstance(node, ast.comprehension) and node.is_async:
            raise ValueError('Python 不支持异步推导式')
        for child in ast.iter_child_nodes(node):
            visit(child, depth+1)
    if sum(1 for _ in ast.walk(tree)) > 1000:
        raise ValueError('Python 代码过于复杂')
    visit(tree)
    return tree


def run_code(source, variables, joints, checkpoint=lambda: None):
    tree = validate_code(source)
    context = {'variables': {k: checked_value(v) for k, v in variables.items()},
               'joints': checked_value(joints)}
    output, work, line = [], 0, 1
    deadline = time.monotonic() + 2

    def tick(node):
        nonlocal work, line
        work += 1
        line = getattr(node, 'lineno', line)
        checkpoint()
        if work > 10000 or time.monotonic() > deadline:
            raise ValueError('Python 计算超过 10000 步或 2 秒限制')

    def iterable(value):
        if not isinstance(value, (list, tuple, dict, str, range)) or len(value) > 256:
            raise ValueError('Python 循环或集合输入最多 256 项')
        return value

    def binary(op, a, b):
        if type(op) in (ast.Add, ast.Mult) and isinstance(a, (str, list)):
            if isinstance(op, ast.Mult):
                if type(b) is not int or not 0 <= b <= 4096 or len(a)*b > (4096 if isinstance(a, str) else 256):
                    raise ValueError('Python 序列乘法结果过大')
            elif type(a) is not type(b):
                raise ValueError('Python 相加的序列类型必须相同')
        elif type(a) not in (int, float, bool) or type(b) not in (int, float, bool):
            raise ValueError('Python 此运算需要数值')
        if isinstance(op, ast.Pow) and abs(b) > 16:
            raise ValueError('Python 幂运算指数绝对值最多为 16')
        return checked_value(BINARY[type(op)](a, b))

    def assign(target, value):
        tick(target)
        if isinstance(target, ast.Name):
            if target.id in RESERVED or len(context) >= 64 and target.id not in context:
                raise ValueError('Python 不能覆盖内置名称，局部变量最多 64 个')
            checked_value(value)
            context[target.id] = value
        elif isinstance(target, (ast.Tuple, ast.List)):
            items = iterable(value)
            if len(items) != len(target.elts):
                raise ValueError('Python 解包数量不匹配')
            for child, item in zip(target.elts, items):
                assign(child, item)
        elif isinstance(target, ast.Subscript):
            container, key = evaluate(target.value), subscript(target.slice)
            if isinstance(container, list):
                if type(key) is not int or not -len(container) <= key < len(container):
                    raise ValueError('Python 列表下标越界或不是整数')
            elif isinstance(container, dict):
                name(key)
                if key not in container and len(container) >= 64:
                    raise ValueError('Python 字典最多 64 个字段')
            else:
                raise ValueError('Python 赋值目标需要列表或字典')
            checked_value(value)
            container[key] = value
            if container is not context['variables']:
                checked_value(container)
        else:
            raise ValueError('Python 不支持此赋值目标')

    def subscript(node):
        if isinstance(node, ast.Slice):
            return slice(*(evaluate(v) if v else None for v in (node.lower, node.upper, node.step)))
        return evaluate(node)

    def call(node):
        args = [evaluate(v) for v in node.args]
        if isinstance(node.func, ast.Name):
            key = node.func.id
            if key == 'print':
                text = ' '.join(str(v) for v in args)
                if len(output) >= 50 or sum(map(len, output))+len(text) > 4096:
                    raise ValueError('Python print 输出过多')
                output.append(text)
                return None
            if key == 'range':
                result = range(*args)
                iterable(result)
                return result
            if key == 'enumerate':
                result = list(enumerate(iterable(args[0]), *args[1:]))
            elif key == 'zip':
                result = list(zip(*(iterable(v) for v in args)))
            elif key in FUNCTIONS:
                if key in ('list', 'sorted') and args:
                    iterable(args[0])
                result = FUNCTIONS[key](*args)
            else:
                raise ValueError(f'Python 不支持调用：{key}')
            return checked_value(result)
        if isinstance(node.func, ast.Attribute):
            attr = node.func
            if isinstance(attr.value, ast.Name) and attr.value.id == 'math' and attr.attr in MATH_FUNCTIONS:
                if any(type(v) not in (int, float) for v in args):
                    raise ValueError('math 函数只接受数值')
                return checked_value(MATH_FUNCTIONS[attr.attr](*args))
            container = evaluate(attr.value)
            if attr.attr == 'append' and isinstance(container, list) and len(args) == 1:
                if len(container) >= 256:
                    raise ValueError('Python 列表最多 256 项')
                checked_value(args[0])
                container.append(args[0])
                checked_value(container)
                return None
            if attr.attr == 'get' and isinstance(container, dict) and 1 <= len(args) <= 2:
                name(args[0])
                return container.get(*args)
        raise ValueError('Python 不支持此函数或方法')

    def evaluate(node):
        tick(node)
        if isinstance(node, ast.Constant):
            return checked_value(node.value)
        if isinstance(node, ast.Name):
            if node.id not in context:
                raise ValueError(f'Python 变量未定义：{node.id}')
            return context[node.id]
        if isinstance(node, (ast.List, ast.Tuple)):
            return checked_value([evaluate(v) for v in node.elts])
        if isinstance(node, ast.Dict):
            result = {}
            for key, value in zip(node.keys, node.values):
                if key is None:
                    raise ValueError('Python 不支持字典展开')
                result[name(evaluate(key))] = evaluate(value)
            return checked_value(result)
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == 'math' and node.attr in MATH_CONSTANTS:
            return MATH_CONSTANTS[node.attr]
        if isinstance(node, ast.Subscript):
            container, key = evaluate(node.value), subscript(node.slice)
            if not isinstance(container, (list, tuple, dict, str)):
                raise ValueError('Python 下标读取只接受列表、字典或文本')
            try:
                return container[key]
            except IndexError as exc:
                raise ValueError('Python 列表或文本下标越界') from exc
            except KeyError as exc:
                raise ValueError(f'Python 字典不存在字段：{key}') from exc
        if isinstance(node, ast.BinOp):
            return binary(node.op, evaluate(node.left), evaluate(node.right))
        if isinstance(node, ast.UnaryOp):
            operand = evaluate(node.operand)
            if isinstance(node.op, ast.Not):
                return not operand
            if type(operand) not in (int, float, bool):
                raise ValueError('Python 正负号只接受数值')
            return checked_value(-operand if isinstance(node.op, ast.USub) else +operand)
        if isinstance(node, ast.BoolOp):
            for item in node.values:
                result = evaluate(item)
                if bool(result) == isinstance(node.op, ast.Or):
                    break
            return result
        if isinstance(node, ast.Compare):
            first = evaluate(node.left)
            for op, item in zip(node.ops, node.comparators):
                second = evaluate(item)
                if not COMPARE[type(op)](first, second):
                    return False
                first = second
            return True
        if isinstance(node, ast.IfExp):
            return evaluate(node.body if evaluate(node.test) else node.orelse)
        if isinstance(node, ast.Call):
            return call(node)
        if isinstance(node, ast.ListComp):
            result = []
            before = dict(context)
            def collect(level):
                if level == len(node.generators):
                    result.append(evaluate(node.elt))
                    checked_value(result)
                    return
                generator = node.generators[level]
                for item in iterable(evaluate(generator.iter)):
                    tick(generator)
                    assign(generator.target, item)
                    if all(evaluate(test) for test in generator.ifs):
                        collect(level+1)
            try:
                collect(0)
            finally:
                context.clear(); context.update(before)
            return result
        raise ValueError(f'Python 不支持此表达式：{type(node).__name__}')

    def statements(items):
        for node in items:
            tick(node)
            if isinstance(node, ast.Assign):
                value = evaluate(node.value)
                for target in node.targets:
                    assign(target, value)
            elif isinstance(node, ast.AugAssign):
                assign(node.target, binary(node.op, evaluate(node.target), evaluate(node.value)))
            elif isinstance(node, ast.Expr):
                evaluate(node.value)
            elif isinstance(node, ast.If):
                statements(node.body if evaluate(node.test) else node.orelse)
            elif isinstance(node, ast.For):
                for item in list(iterable(evaluate(node.iter))):
                    tick(node)
                    assign(node.target, item)
                    statements(node.body)
                statements(node.orelse)
            elif not isinstance(node, (ast.Import, ast.Pass)):
                raise ValueError(f'Python 不支持此语句：{type(node).__name__}')
    try:
        statements(tree.body)
        if 'result' not in context:
            raise ValueError('Python 代码必须给 result 赋值')
        result = checked_value(context['result'])
        updated = context['variables']
        if len(updated) > 64:
            raise ValueError('最多 64 个程序变量')
        updated = {name(k): checked_value(v) for k, v in updated.items()}
        return result, updated, output
    except InterruptedError:
        raise
    except (ValueError, TypeError, ArithmeticError, LookupError, RuntimeError) as exc:
        if isinstance(exc, ZeroDivisionError):
            exc = ValueError("除数不能为 0")
        raise ValueError(f'Python 第 {line} 行：{exc}') from exc
