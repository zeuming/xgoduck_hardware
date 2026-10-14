"""Bounded read-only log pagination and real HTTP authentication/isolation."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import urllib.request

import native_process as app
import test_native_process as process_tests


class LogReaderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name).resolve() / 'native-evidence'
        self.directory.mkdir(mode=0o700)
        self.reader = app.LogReader(self.directory)

    def tearDown(self):
        self.tmp.cleanup()

    def record(self, index, **changes):
        path = self.directory / ('operation-%016x.json' % index)
        value = dict(action='start', state='complete', mode='hold',
                     secret='must-not-leak', before={'token': 'private'})
        value.update(changes)
        path.write_text(json.dumps(value))
        path.chmod(0o600)
        stamp = 1700000000000000000 + index * 1000000000
        os.utime(path, ns=(stamp, stamp))
        return path

    def test_latest_and_stable_cursor_do_not_repeat_on_new_records(self):
        for index in range(20):
            self.record(index)
        first = self.reader.page()
        self.assertEqual(len(first['items']), 8)
        self.assertEqual([app.LogReader.decode_cursor(item['id'])[1] for item in first['items']],
                         ['operation-%016x.json' % index for index in range(19, 11, -1)])
        self.record(99)
        self.reader.index_at = -float('inf')
        second = self.reader.page(first['next_cursor'])
        self.assertEqual([app.LogReader.decode_cursor(item['id'])[1] for item in second['items']],
                         ['operation-%016x.json' % index for index in range(11, 3, -1)])
        last = self.reader.page(second['next_cursor'])
        self.assertEqual(len(last['items']), 4)
        self.assertIsNone(last['next_cursor'])
        serialized = json.dumps(first)
        self.assertNotIn('private', serialized)
        self.assertNotIn('must-not-leak', serialized)

    def test_symlink_and_large_records_are_never_read(self):
        secret = self.directory.parent / 'secret'
        secret.write_text('do not read')
        (self.directory / ('operation-' + 'a' * 16 + '.json')).symlink_to(secret)
        large = self.record(1)
        large.write_bytes(b'x' * (self.reader.MAX_FILE_BYTES + 1))
        (self.directory / 'live-state.json').write_text('{"token":"private"}')
        with mock.patch.object(app.os, 'open', side_effect=AssertionError('Must not open large/link files')):
            page = self.reader.page()
        self.assertEqual(len(page['items']), 1)
        self.assertIn('较大', page['items'][0]['message'])

    def test_invalid_cursor_and_busy_reader_never_read_files(self):
        for value in ('../secret', 'A' * 241, app.LogReader.cursor((1, '../secret')),
                      app.LogReader.cursor(('1', 'operation-' + 'a' * 16 + '.json'))):
            with self.assertRaises(ValueError):
                self.reader.page(value)
        self.reader.lock.acquire()
        try:
            with self.assertRaisesRegex(ValueError, '正在读取'):
                self.reader.page()
        finally:
            self.reader.lock.release()

    def test_scan_bound_is_reported_and_cached_summaries_avoid_repeat_reads(self):
        for index in range(12):
            self.record(index)
        first = self.reader.page()
        with mock.patch.object(app.os, 'open', side_effect=AssertionError('Cached page reread records')):
            self.assertEqual(self.reader.page()['items'], first['items'])
        self.reader.MAX_FILES = 2
        self.reader.index_at = -float('inf')
        self.assertTrue(self.reader.page()['index_truncated'])


class LogHTTPTests(process_tests.ProcessTests):
    # Use the actual separate HTTP subprocess from the existing fixture.
    def test_log_routes_read_disk_without_rpc_or_controller_snapshot(self):
        directory = self.root / 'native-evidence'
        directory.mkdir(mode=0o700, exist_ok=True)
        for index in range(10):
            path = directory / ('operation-%016x.json' % index)
            path.write_text(json.dumps(dict(action='stop', state='complete', token='private')))
            path.chmod(0o600)
            stamp = 1700000000000000000 + index * 1000000000
            os.utime(path, ns=(stamp, stamp))
        self.bridge.stop.set()
        self.bridge.threads[0].join(1)
        with mock.patch.object(self.authority, 'snapshot', side_effect=AssertionError('Logs read controller')):
            code, first = self.request('/api/logs/latest')
            self.assertEqual(code, 200, first)
            self.assertEqual(len(first['items']), 8)
            self.assertNotIn('private', json.dumps(first))
            code, second = self.request('/api/logs/before/' + first['next_cursor'])
            self.assertEqual(code, 200, second)
            self.assertEqual(len(second['items']), 2)
            self.assertEqual(self.runtime.calls, [])
        outsider = urllib.request.build_opener()
        self.assertEqual(self.request('/api/logs/latest', client=outsider)[0], 401)
        self.assertEqual(self.request('/api/logs/before/invalid')[0], 409)
        code, page = self.request('/')
        self.assertEqual(code, 200)
        self.assertIn(b'runtimeLogPanel', page)


if __name__ == '__main__':
    unittest.main()
