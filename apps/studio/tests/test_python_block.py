"""Python calculation and diagnostic regression checks; no hardware access."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import ProgramError, Session, compile_program
from python_block import run_code, validate_code
from robot import READY


class PythonBlockTests(unittest.TestCase):
    def test_python_calculation_variables_loops_and_output(self):
        variables = {'目标': READY.copy(), '次数': 2}
        code = '''import math
result = list(variables["目标"])
for i, q in enumerate(result):
    if i == 0:
        result[i] += math.radians(variables["次数"])
variables["状态"] = "已计算"
print("计算完成", len(result))'''
        result, updated, output = run_code(code, variables, READY)
        self.assertAlmostEqual(result[0], READY[0]+2*3.141592653589793/180)
        self.assertEqual(updated['状态'], '已计算')
        self.assertEqual(variables, {'目标': READY, '次数': 2})
        self.assertEqual(output, ['计算完成 6'])
        result, _, _ = run_code('result = [round(q, 4) for q in joints if q < 1]', {}, READY)
        self.assertEqual(result, [round(q, 4) for q in READY if q < 1])
        result, _, _ = run_code('result = []\nfor i in range(3):\n    result.append(i**2)\nresult = result[::-1]', {}, READY)
        self.assertEqual(result, [4, 1, 0])
        result, _, _ = run_code('result = {"sum": sum([1, 2, 3]), "ok": 1 < 2 < 3}\nresult = result.get("sum") if result["ok"] else 0', {}, READY)
        self.assertEqual(result, 6)
        result, updated, _ = run_code('result = [1, 2]\nalias = result\nvariables["目标"] = result\nalias[0] = 3', {}, READY)
        self.assertEqual(result, [3, 2])
        self.assertEqual(updated['目标'], [3, 2])

    def test_forbidden_syntax_and_capabilities(self):
        for code in ('import os\nresult = 1', 'from math import sin\nresult = 1',
                     'result = __import__("os")', 'result = joints.__class__',
                     'while True:\n    pass', 'def f():\n    pass',
                     'result = open("/tmp/should-never-be-opened")',
                     'result = eval("1+1")', 'result = math.__dict__'):
            with self.subTest(code=code), self.assertRaises(ValueError):
                run_code(code, {}, READY)

    def test_errors_are_bounded_and_keep_line_numbers(self):
        for code, message in (
            ('result = 1\nresult = 1 / 0', '第 2 行'),
            ('result = joints[100]', '下标越界'),
            ('result = missing', '变量未定义'),
            ('x = 1', 'result'),
            ('result = list(range(1000000))', '256'),
            ('result = "x" * 1000000000', '过大'),
            ('result = 2 ** 1000000', '指数'),
            ('for i in range(256):\n    for j in range(256):\n        pass\nresult = 0', '10000'),
            ('for i in range(51):\n    print(i)\nresult = 1', '输出过多'),
            ('result = float("nan")', '范围'),
            ('result = []\nresult.append(result)', '过深'),
        ):
            with self.subTest(code=code), self.assertRaisesRegex(ValueError, message):
                run_code(code, {}, READY)
        with self.assertRaisesRegex(ValueError, '语法错误，第 1 行'):
            validate_code('result = [')
        with self.assertRaisesRegex(ValueError, '8192'):
            validate_code('x'*8193)
        def stop():
            raise InterruptedError('已停止')
        with self.assertRaises(InterruptedError):
            run_code('result = 1', {}, READY, stop)

    def test_program_errors_point_to_the_deepest_action(self):
        class Model:
            limits = [(-3, 4)]*6
            def check_path(self, *_args):
                pass
        session = Session(Model())
        session.connect('simulation'); session.enable(True)
        target = READY.copy(); target[0] += .01
        program = [dict(id='move', type='move', joints=target, duration=.2),
                   dict(id='python', type='python', name='计算结果', code='result = 1 / 0')]
        session.start(compile_program(program, {}, Model.limits))
        session.worker.join(2)
        self.assertEqual(session.phase, 'error')
        self.assertEqual(session.state()['error_block'], 'python')
        self.assertEqual(session.q, READY)
        for item in (dict(id='variable', type='show', value=dict(type='variable', name='missing')),
                     dict(id='condition', type='if', condition=dict(type='literal', value=1), then=[], **{'else': []})):
            with self.subTest(item=item), self.assertRaises(ProgramError) as error:
                session.prepare(compile_program([item], {}, Model.limits))
            self.assertEqual(error.exception.block_id, item['id'])
        with self.assertRaises(ProgramError) as error:
            compile_program([dict(id='call', type='call', name='missing')], {}, Model.limits)
        self.assertEqual(error.exception.block_id, 'call')
        prepared = session.prepare(compile_program([
            dict(id='python', type='python', name='计算结果', code='variables["目标"] = list(joints)\nresult = [q for q in joints]'),
            dict(id='move', type='move_value', value=dict(type='variable', name='计算结果'), duration=.2),
        ], {}, Model.limits))
        self.assertEqual(prepared[-1]['joints'], READY)


if __name__ == '__main__':
    unittest.main()
