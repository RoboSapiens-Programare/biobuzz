import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

class _FakeArray(list):
    @property
    def size(self):
        return len(self)


if "numpy" not in sys.modules:
    sys.modules["numpy"] = types.SimpleNamespace(
        array=lambda values, dtype=None: _FakeArray(values),
        polyfit=lambda *args, **kwargs: [],
        nan=float("nan"),
    )
if "matplotlib" not in sys.modules:
    matplotlib = types.ModuleType("matplotlib")
    matplotlib.use = lambda *args, **kwargs: None
    sys.modules["matplotlib"] = matplotlib
    pyplot = types.ModuleType("matplotlib.pyplot")
    sys.modules["matplotlib.pyplot"] = pyplot

MODULE_PATH = Path(__file__).resolve().parent / "train_polynomial_regression.py"
SPEC = importlib.util.spec_from_file_location("train_polynomial_regression", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class JavaSnippetTests(unittest.TestCase):
    def test_degree_zero(self):
        self.assertIn("return computeVelocityC0;", MODULE.java_snippet([1.0]))

    def test_degree_one(self):
        self.assertIn(
            "return computeVelocityC0 + dist * computeVelocityC1;",
            MODULE.java_snippet([1.0, 2.0]),
        )

    def test_degree_two(self):
        self.assertIn(
            "return computeVelocityC0 + dist * (computeVelocityC1 + dist * computeVelocityC2);",
            MODULE.java_snippet([1.0, 2.0, 3.0]),
        )

    def test_degree_three_with_custom_method_name(self):
        self.assertIn(
            "return computeHoodC0 + dist * (computeHoodC1 + dist * (computeHoodC2 + dist * computeHoodC3));",
            MODULE.java_snippet([1.0, 2.0, 3.0, 4.0], method_name="computeHood"),
        )


class LoadDataTests(unittest.TestCase):
    def test_load_data_reads_selected_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.json"
            path.write_text(json.dumps([{"dist": 1.0, "vel": 2.0}, {"dist": 3.0, "vel": 4.0}, {"dist": 5.0, "vel": 6.0}]))
            xs, ys = MODULE.load_data(str(path), "dist", "vel")
            self.assertEqual(xs, [1.0, 3.0, 5.0])
            self.assertEqual(ys, [2.0, 4.0, 6.0])

    def test_load_data_rejects_too_few_points(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.json"
            path.write_text(json.dumps([{"dist": 1.0, "vel": 2.0}]))
            with self.assertRaises(ValueError):
                MODULE.load_data(str(path), "dist", "vel")


if __name__ == "__main__":
    unittest.main()
