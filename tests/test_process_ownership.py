"""Real local file leases; no USB, native commands or ROS processes."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from osmo360_driver.process_ownership import (
    InvalidOwnershipFileError, OwnershipBusyError, process_ownership,
)


class ProcessOwnershipTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / 'camera.lock'

    def test_busy_owner_is_rejected_without_replacing_or_removing_lock(self):
        with process_ownership(self.path):
            inode = self.path.stat().st_ino
            with self.assertRaises(OwnershipBusyError):
                with process_ownership(self.path):
                    self.fail('A second camera owner entered')
            self.assertEqual(self.path.stat().st_ino, inode)
        self.assertTrue(self.path.is_file())
        with process_ownership(self.path):
            self.assertEqual(self.path.stat().st_ino, inode)

    def test_exception_releases_lease_and_preserves_original_error(self):
        original = ValueError('camera initialization failed')
        with self.assertRaises(ValueError) as caught:
            with process_ownership(self.path):
                raise original
        self.assertIs(caught.exception, original)
        with process_ownership(self.path):
            pass

    def test_another_process_observes_busy_then_acquires_after_release(self):
        command = '''
import sys
from osmo360_driver.process_ownership import process_ownership, OwnershipBusyError
try:
    with process_ownership(sys.argv[1]):
        print('acquired')
except OwnershipBusyError:
    print('busy')
'''
        def child():
            return subprocess.run([sys.executable, '-c', command, str(self.path)],
                                  check=True, capture_output=True, text=True, timeout=10).stdout.strip()
        with process_ownership(self.path):
            self.assertEqual(child(), 'busy')
        self.assertEqual(child(), 'acquired')

    def test_symlink_is_never_followed(self):
        target = self.path.with_name('target')
        target.write_text('unchanged')
        self.path.symlink_to(target)
        with self.assertRaises(OSError):
            with process_ownership(self.path):
                self.fail('A symlink became the camera lock')
        self.assertEqual(target.read_text(), 'unchanged')

    def test_process_exit_without_python_cleanup_does_not_leave_a_stale_owner(self):
        command = '''
import os, sys
from osmo360_driver.process_ownership import process_ownership
with process_ownership(sys.argv[1]):
    os._exit(0)
'''
        subprocess.run([sys.executable, '-c', command, str(self.path)],
                       check=True, capture_output=True, text=True, timeout=10)
        self.assertTrue(self.path.is_file())
        with process_ownership(self.path):
            pass

    def test_hardlinked_file_is_rejected(self):
        self.path.touch()
        os.link(self.path, self.path.with_name('alias'))
        with self.assertRaises(InvalidOwnershipFileError):
            with process_ownership(self.path):
                self.fail('A hardlinked file became the camera lock')

    def test_missing_parent_is_not_created_implicitly(self):
        path = self.path.parent / 'missing' / 'camera.lock'
        with self.assertRaises(FileNotFoundError):
            with process_ownership(path):
                self.fail('The driver created application directories')
        self.assertFalse(path.parent.exists())


if __name__ == '__main__':
    unittest.main()
