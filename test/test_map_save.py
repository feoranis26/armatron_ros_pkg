import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from armatron.map_manager import command_save
from armatron.map_state import save_state


class MapSaveTest(unittest.TestCase):
    def run_save(self, output, complete=True, code=0, success=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            save_state({'active_profile': 'primary', 'mode': 'mapping'}, root)
            profile = root / 'maps/primary'
            current = profile / 'current'
            current.mkdir(parents=True)
            (current / 'map.posegraph').write_text('old graph')
            (current / 'map.data').write_text('old data')

            def serialize(*args, **kwargs):
                (profile / '.staging/map.posegraph').write_text('new graph')
                if complete:
                    (profile / '.staging/map.data').write_text('new data')
                return SimpleNamespace(returncode=code, stdout=output, stderr='')

            with patch('armatron.map_manager.subprocess.run', side_effect=serialize), \
                 patch('armatron.map_manager.export_grid'):
                if success:
                    command_save(SimpleNamespace(root=root, timeout=20))
                    self.assertEqual((current / 'map.data').read_text(), 'new data')
                    self.assertEqual((profile / 'previous/map.data').read_text(), 'old data')
                else:
                    with self.assertRaises(RuntimeError):
                        command_save(SimpleNamespace(root=root, timeout=20))
                    self.assertEqual((current / 'map.data').read_text(), 'old data')
                    self.assertTrue((profile / '.staging/map.posegraph').exists())

    def test_humble_response_promotes_complete_map(self):
        self.run_save('response:\nslam_toolbox.srv.SerializePoseGraph_Response(result=0)\n')

    def test_yaml_response_promotes_complete_map(self):
        self.run_save('response:\nresult: 0\n')

    def test_failure_and_missing_artifacts_preserve_previous_map(self):
        for output, complete, code in [
            ('response:\nSerializePoseGraph_Response(result=1)', True, 0),
            ('response:\nSerializePoseGraph_Response(result=0)', False, 0),
            ('response:\nSerializePoseGraph_Response(result=0)', True, 1),
            ('requester: filename=result: 0', True, 0),
        ]:
            with self.subTest(output=output, complete=complete, code=code):
                self.run_save(output, complete, code, success=False)
