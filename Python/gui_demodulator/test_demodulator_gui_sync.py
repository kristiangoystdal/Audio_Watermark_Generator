#!/usr/bin/env python3

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np
import soundfile as sf

from demodulator_gui import (
    derive_synced_txt_path,
    derive_synced_wav_path,
    find_shared_message_timestamp,
    parse_label_entries,
    synchronize_audio_files,
)


class DemodulatorGuiSyncTests(unittest.TestCase):
    def test_parse_label_entries_extracts_message_timestamp(self):
        with TemporaryDirectory() as tmpdir:
            txt_path = Path(tmpdir) / "sample.txt"
            txt_path.write_text(
                "1.250000\t2.500000\tTime: 15:17:01 on Wednesday 18/03/2026 | Message: Hello\n",
                encoding="utf-8",
            )

            entries = parse_label_entries(str(txt_path))

            self.assertEqual(len(entries), 1)
            self.assertAlmostEqual(entries[0].start_time, 1.25)
            self.assertEqual(entries[0].message_timestamp.isoformat(), "2026-03-18T15:17:01")

    def test_find_shared_message_timestamp_uses_common_timestamp(self):
        with TemporaryDirectory() as tmpdir:
            first = Path(tmpdir) / "first.txt"
            second = Path(tmpdir) / "second.txt"
            first.write_text(
                "\n".join(
                    [
                        "1.000000\t1.500000\tTime: 12:00:05 on Wednesday 18/03/2026 | Message: One",
                        "2.000000\t2.500000\tTime: 12:00:10 on Wednesday 18/03/2026 | Message: Two",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            second.write_text(
                "\n".join(
                    [
                        "3.000000\t3.500000\tTime: 12:00:10 on Wednesday 18/03/2026 | Message: Two",
                        "4.000000\t4.500000\tTime: 12:00:20 on Wednesday 18/03/2026 | Message: Three",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            shared = find_shared_message_timestamp(
                {
                    "first": parse_label_entries(str(first)),
                    "second": parse_label_entries(str(second)),
                }
            )

            self.assertIsNotNone(shared)
            self.assertEqual(shared.isoformat(), "2026-03-18T12:00:10")

    def test_synchronize_audio_files_creates_equal_length_synced_outputs(self):
        with TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            sample_rate = 8000
            wav_a = root / "alpha.wav"
            wav_b = root / "beta.wav"

            audio_a = np.linspace(-0.5, 0.5, sample_rate * 5, dtype=np.float32)
            audio_b = np.linspace(0.5, -0.5, sample_rate * 7, dtype=np.float32)
            sf.write(wav_a, audio_a, sample_rate)
            sf.write(wav_b, audio_b, sample_rate)

            (root / "alpha.txt").write_text(
                "\n".join(
                    [
                        "1.000000\t1.500000\tTime: 12:00:00 on Wednesday 18/03/2026 | Message: Shared",
                        "2.000000\t2.500000\tTime: 12:00:20 on Wednesday 18/03/2026 | Message: Alpha only",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            (root / "beta.txt").write_text(
                "\n".join(
                    [
                        "3.000000\t3.500000\tTime: 12:00:00 on Wednesday 18/03/2026 | Message: Shared",
                        "4.000000\t4.500000\tTime: 12:00:30 on Wednesday 18/03/2026 | Message: Beta only",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = synchronize_audio_files([str(wav_a), str(wav_b)])

            self.assertEqual(len(result.outputs), 2)

            info_a = sf.info(derive_synced_wav_path(str(wav_a)))
            info_b = sf.info(derive_synced_wav_path(str(wav_b)))
            self.assertEqual(info_a.frames, info_b.frames)
            self.assertEqual(info_a.frames, sample_rate * 4)

            synced_entries_a = parse_label_entries(derive_synced_txt_path(str(wav_a)))
            synced_entries_b = parse_label_entries(derive_synced_txt_path(str(wav_b)))
            self.assertEqual(len(synced_entries_a), 2)
            self.assertEqual(len(synced_entries_b), 2)
            self.assertAlmostEqual(synced_entries_a[0].start_time, 0.0, places=6)
            self.assertAlmostEqual(synced_entries_b[0].start_time, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
