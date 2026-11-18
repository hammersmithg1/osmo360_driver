"""GUI evidence uses a post-read upper bound and preserves all strict gates."""

import importlib.util
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[1] / 'osmo360_camera/_device/usb_standby_owner.py'
SPEC = importlib.util.spec_from_file_location('standby_gui_cursor_tests', SOURCE)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


class GuiCursorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'gui.log'
        self.path.write_bytes(b'')
        # Production demands root. Adapt only fixture uid on non-root hosts;
        # actual inode, regular-file, size, no-follow and pread checks remain.
        fstat = os.fstat
        def root_owned(fd):
            info = fstat(fd)
            return SimpleNamespace(st_uid=0, st_mode=info.st_mode, st_dev=info.st_dev,
                                   st_ino=info.st_ino, st_size=info.st_size)
        owned = patch.object(m.os, 'fstat', side_effect=root_owned)
        owned.start()
        self.addCleanup(owned.stop)
        self.cursor = m.GuiCursor(self.path)
        self.addCleanup(self.cursor.close)
        self.base = time.mktime((2026, 11, 18, 4, 46, 28, 0, 0, -1))

    def line(self, milliseconds, marker):
        stamp = time.strftime('%m-%d %H:%M:%S', time.localtime(self.base))
        return f'{stamp}.{milliseconds:03d}   776 1794 I GUI: {marker}\n'.encode()

    def append(self, *lines):
        with self.path.open('ab') as stream:
            stream.write(b''.join(lines))

    def test_callbacks_appended_after_poll_begins_survive_delayed_pread(self):
        clock, events = [self.base + .005], []
        stat, pread = Path.stat, os.pread
        def during_stat(path, *args, **options):
            if path == self.path:
                # Real writer interleaving: append AFTER a pre-read clock
                # sample would have happened, BEFORE fstat supplies read size.
                clock[0] = self.base + .012
                self.append(self.line(10, m.PROCESSING), self.line(11, m.DONE))
                events.append('append')
            return stat(path, *args, **options)
        def delayed_pread(fd, count, offset):
            data = pread(fd, count, offset)
            clock[0] = self.base + .013
            events.append('pread_return')
            return data
        def now():
            events.append('clock')
            return clock[0]
        with (patch.object(Path, 'stat', autospec=True, side_effect=during_stat),
              patch.object(m.os, 'pread', side_effect=delayed_pread),
              patch.object(m.time, 'time', side_effect=now)):
            self.assertTrue(self.cursor.poll(self.base + .001))
        self.assertEqual(events, ['append', 'pread_return', 'clock'])
        self.assertEqual([row['kind'] for row in self.cursor.found], ['processing3', 'done3'])
        self.assertTrue(self.cursor.proof()['done3_after_intent'])
        self.assertEqual(self.cursor.offset, self.path.stat().st_size)

    def test_after_read_upper_bound_still_rejects_genuinely_future_events(self):
        self.append(self.line(20, m.PROCESSING), self.line(21, m.DONE))
        pread, clock = os.pread, [self.base + .005]
        def delayed(fd, count, offset):
            data = pread(fd, count, offset)
            clock[0] = self.base + .010
            return data
        with patch.object(m.os, 'pread', side_effect=delayed), patch.object(m.time, 'time', side_effect=lambda: clock[0]):
            self.assertFalse(self.cursor.poll(self.base))
        self.assertEqual(self.cursor.found, [])
        self.assertFalse(self.cursor.proof()['processing3_after_intent'])

    def test_buffered_stale_events_before_intent_are_still_rejected(self):
        self.append(self.line(10, m.PROCESSING), self.line(11, m.DONE))
        self.assertFalse(self.cursor.poll(self.base + .100, now=self.base + .200))
        self.assertEqual(self.cursor.found, [])

    def test_processing_then_done_order_is_still_required(self):
        self.append(self.line(10, m.DONE), self.line(11, m.PROCESSING))
        self.assertFalse(self.cursor.poll(self.base, now=self.base + .100))
        self.assertEqual([row['kind'] for row in self.cursor.found], ['processing3'])
        self.append(self.line(12, m.DONE))
        self.assertTrue(self.cursor.poll(self.base, now=self.base + .100))

    def test_rotation_or_replacement_is_not_adopted(self):
        self.path.rename(self.path.with_suffix('.log.1'))
        self.path.write_bytes(self.line(10, m.PROCESSING) + self.line(11, m.DONE))
        with self.assertRaisesRegex(ValueError, 'rotated/replaced/truncated'):
            self.cursor.poll(self.base, now=self.base + 1)

    def test_truncation_is_still_terminal(self):
        self.append(b'old line\n')
        self.cursor.poll(self.base, now=self.base)
        self.path.write_bytes(b'')
        with self.assertRaisesRegex(ValueError, 'rotated/replaced/truncated'):
            self.cursor.poll(self.base, now=self.base)

    def test_max_log_bound_still_precedes_read(self):
        self.append(b'x' * (m.MAX_LOG + 1))
        with patch.object(m.os, 'pread', side_effect=AssertionError('unbounded read')) as read:
            with self.assertRaisesRegex(ValueError, 'exceeds1MiB'):
                self.cursor.poll(self.base, now=self.base)
        read.assert_not_called()

    def test_unbounded_partial_line_still_fails(self):
        self.append(b'x' * 8193)
        with self.assertRaisesRegex(ValueError, 'Unbounded GUI log line'):
            self.cursor.poll(self.base, now=self.base)


if __name__ == '__main__':
    unittest.main()
