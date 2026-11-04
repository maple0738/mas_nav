import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET

from create_undulating_sim import build


class UndulatingArena(unittest.TestCase):
    def test_collision_and_visual_use_measured_wave_mesh(self):
        simulator=Path('/home/jsy/Projects/navigation/simulation/rmu-gazebo-simulator')
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)
            data=build(output,simulator,math.pi/4,closed_lane=True)
            model=output/'rmu_gazebo_simulator/resource/models/undulating_test'
            blob=(model/'road.stl').read_bytes()
            count=struct.unpack_from('<I',blob,80)[0]
            self.assertEqual(len(blob),84+count*50)
            top={}
            for offset in range(84,len(blob),50):
                values=struct.unpack_from('<12fH',blob,offset)
                for j in (3,6,9):
                    x,y,z=values[j:j+3]
                    if z>.03:top[x]=max(top.get(x,-math.inf),z)
            self.assertAlmostEqual(max(top.values())-min(top.values()),.07,places=6)
            peaks=sorted(x for x,z in top.items() if abs(z-.11)<1e-6)
            self.assertEqual(len(peaks),12)
            for a,b in zip(peaks,peaks[1:]):self.assertAlmostEqual(b-a,.24,places=5)
            root=ET.parse(model/'model.sdf').getroot()
            self.assertEqual(root.findtext('model/link/collision/geometry/mesh/uri'),
                             root.findtext('model/link/visual/geometry/mesh/uri'))
            self.assertTrue((model/'model.config').is_file())
            self.assertTrue(data['closed_lane'])
            self.assertEqual(len(root.findall('model/link/collision')),5)
            self.assertAlmostEqual(data['spawn'][0],-1.2/math.sqrt(2))
            self.assertAlmostEqual(data['goal'][1],4.08/math.sqrt(2))


if __name__=='__main__':unittest.main()
