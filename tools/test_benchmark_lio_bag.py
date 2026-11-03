"""Wire-layout regression for the bag converter; run in the active ROS overlay."""
import struct
import unittest
from benchmark_lio_bag import decode_livox, small_parameters


def serialized(frame_id='sensor', tags=(0, 16, 32), final_padding=True):
    # Construct a CDR1 message from the published CustomMsg/CustomPoint definition.
    payload = bytearray(struct.pack('<iI', 7, 123))
    text = frame_id.encode() + b'\0'
    payload.extend(struct.pack('<I', len(text)))
    payload.extend(text)
    while len(payload) % 8:
        payload.append(0)
    payload.extend(struct.pack('<QI', 7_000_000_123, len(tags)))
    payload.extend(b'\0' * 4)  # lidar_id and reserved bytes
    payload.extend(struct.pack('<I', len(tags)))
    for index, tag in enumerate(tags):
        while len(payload) % 4:
            payload.append(0)
        payload.extend(struct.pack('<IfffBBB', index * 1000, index + 1., 2., 3., 5, tag, 0))
    if final_padding:
        while len(payload) % 4:
            payload.append(0)
    return b'\0\1\0\0' + payload


class LivoxWireFormat(unittest.TestCase):
    def test_variable_string_alignment_and_strict_common_tag_filter(self):
        for frame in ['sensor', '', 'long_lidar_frame_name']:
            cloud, raw, count, timebase, span = decode_livox(serialized(frame))
            self.assertEqual(raw, 3)
            self.assertEqual(count, 1)
            self.assertEqual(cloud.width, 1)
            self.assertEqual(len(cloud.data), 24)
            self.assertAlmostEqual(timebase, 7.000000123)
            self.assertAlmostEqual(span, .000002)
            self.assertEqual(struct.unpack_from('<fff', cloud.data), (1., 2., 3.))

    def test_original_offsets_are_absolute_seconds_not_header_offsets(self):
        cloud, _, count, _, _ = decode_livox(serialized(tags=(0, 0), final_padding=False))
        self.assertEqual(count, 2)
        self.assertAlmostEqual(struct.unpack_from('<d', cloud.data, 16)[0], 7.000000123)
        self.assertAlmostEqual(struct.unpack_from('<d', cloud.data, 40)[0], 7.000001123)
        self.assertEqual(cloud.header.stamp.nanosec, 123)

    def test_rejects_wrong_endianness_and_truncated_arrays(self):
        data = serialized()
        with self.assertRaises(ValueError):
            decode_livox(b'\0\0\0\0' + data[4:])
        with self.assertRaises(ValueError):
            decode_livox(data[:-4])

    def test_empty_sequence_keeps_an_empty_cloud(self):
        cloud, raw, valid, _, span = decode_livox(serialized(tags=()))
        self.assertEqual((raw, valid, cloud.width, span), (0, 0, 0, 0))
        self.assertEqual(len(cloud.data), 0)


class BenchmarkConfiguration(unittest.TestCase):
    def test_si_acceleration_has_si_saturation_threshold(self):
        params = small_parameters()
        self.assertAlmostEqual(params['satu_acc'], 3 * 9.81)
        self.assertEqual(params['acc_norm'], 9.81)
        self.assertGreater(params['satu_acc'], 2 * abs(params['gravity'][2]))

    def test_adaptation_switches_are_not_exposed(self):
        params = small_parameters()
        self.assertNotIn('enabled', params['adaptive_sampling'])
        self.assertNotIn('enabled', params['adaptive_timing'])


if __name__ == '__main__':
    unittest.main()
