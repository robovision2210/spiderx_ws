"""m55_rclpy.main(): an expected shutdown during executor.spin() (KeyboardInterrupt, or rclpy's
ExternalShutdownException when its signal handler has already shut the context down) ends main
normally and still runs the cleanup: node.close() (which writes the evidence) and
rclpy.try_shutdown(). Any other exception is not swallowed, and the cleanup still runs.

In-process only: rclpy.init, the executor and rclpy.try_shutdown are replaced; no ROS graph."""

import m55_fixtures as fx
import pytest
import rclpy
import rclpy.executors

from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m55_rclpy as m55r


class FakeNode:
    last = None

    def __init__(self, cfg, library, evidence_dir=None):
        self.node = object()
        self.closed = 0
        FakeNode.last = self

    def close(self):
        self.closed += 1


def patched_main(monkeypatch, exc):
    calls = []

    class Executor:
        def add_node(self, node):
            calls.append('add_node')

        def spin(self):
            calls.append('spin')
            raise exc

    cfg = fx.config()                                   # before load_config is replaced
    monkeypatch.setattr(loc, 'load_config', lambda *a, **k: cfg)
    monkeypatch.setattr(loc, 'build_library', lambda cfg, progress=None: 'library')
    monkeypatch.setattr(m55r, 'LocomotionNode', FakeNode)
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: calls.append('init'))
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda *a, **k: calls.append('try_shutdown'))
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', Executor)
    FakeNode.last = None
    return calls


@pytest.mark.parametrize('exc', [KeyboardInterrupt(), rclpy.executors.ExternalShutdownException()],
                         ids=['KeyboardInterrupt', 'ExternalShutdownException'])
def test_expected_shutdown_returns_normally_and_cleans_up(monkeypatch, exc):
    calls = patched_main(monkeypatch, exc)
    assert m55r.main(['m55_locomotion_node']) == 0
    assert FakeNode.last.closed == 1
    assert calls == ['init', 'add_node', 'spin', 'try_shutdown']


def test_an_unrelated_exception_is_not_swallowed_and_cleanup_still_runs(monkeypatch):
    calls = patched_main(monkeypatch, RuntimeError('unrelated failure'))
    with pytest.raises(RuntimeError, match='unrelated failure'):
        m55r.main(['m55_locomotion_node'])
    assert FakeNode.last.closed == 1
    assert calls == ['init', 'add_node', 'spin', 'try_shutdown']


def test_only_the_two_expected_shutdown_exceptions_are_caught():
    src = open(m55r.__file__).read()
    body = src[src.index('def main('):]
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' in body
    assert 'except Exception' not in body and 'except BaseException' not in body
    assert body.count('except') == 1
