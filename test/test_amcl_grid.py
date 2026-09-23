import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import yaml
from armatron.map_manager import command_save, parser
from armatron.map_state import save_state


class AmclGridTest(unittest.TestCase):
    def test_grid_is_portable_and_retained_on_subsequent_automatic_save(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = root / 'maps' / 'primary'
            profile.mkdir(parents=True)
            save_state({'active_profile':'primary','mode':'mapping'},root)
            def call(command, **kwargs):
                stage = profile / '.staging'
                if 'map_saver_cli' in command:
                    grid = stage / 'grid'
                    (grid/'map.pgm').write_bytes(b'P5 fake-test-image')
                    (grid/'map.yaml').write_text(yaml.safe_dump({'image': str(grid/'map.pgm')}))
                    return NS(returncode=0,stdout='',stderr='')
                (stage/'map.posegraph').write_text('graph')
                (stage/'map.data').write_text('data')
                return NS(returncode=0,stdout='response:\nSerializePoseGraph_Response(result=0)',stderr='')
            with patch('armatron.map_manager.subprocess.run', side_effect=call) as run:
                command_save(NS(root=root,timeout=20,with_grid=True))
                command_save(NS(root=root,timeout=20))
                self.assertEqual(run.call_count,4)
                grid = profile/'current/grid'
                self.assertEqual(yaml.safe_load((grid/'map.yaml').read_text())['image'],'map.pgm')
                self.assertTrue((profile/'previous/grid/map.pgm').is_file())
                with patch('armatron.map_manager.export_grid', side_effect=RuntimeError('export failed')):
                    with self.assertRaisesRegex(RuntimeError,'export failed'):
                        command_save(NS(root=root,timeout=20))
                self.assertTrue((grid/'map.pgm').is_file())

    def test_cli_modes(self):
        self.assertEqual(parser().parse_args(['set-mode','amcl']).mode,'amcl')
        self.assertTrue(parser().parse_args(['save','--with-grid']).with_grid)
        self.assertEqual(parser().parse_args(['global-localize']).command,'global-localize')
