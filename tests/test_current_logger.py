from __future__ import annotations

from datetime import datetime, timedelta

from gloria_m_sdk.demo_support.current_logger import CurrentLogWriter


def test_current_log_streams_to_active_file_before_finalize(tmp_path) -> None:
    writer = CurrentLogWriter(
        demo_name="01_gripper_quicktest",
        log_root=tmp_path,
        flush_interval_s=0.0,
    )

    active_path = writer.append_sample(100.0, -1.25)
    writer.append_sample(100.5, 2.5)
    writer.flush()

    assert active_path is not None
    assert active_path.exists()
    assert active_path.name.startswith("01_gripper_quicktest_active_")
    assert active_path.read_text(encoding="utf-8").splitlines() == [
        "elapsed_s,current_a,recorded_duration_s",
        "0.000000,-1.250000,0.000000",
        "0.500000,+2.500000,0.500000",
    ]

    final_path = writer.finalize(ended_at=datetime(2026, 6, 11, 18, 30, 15))
    assert final_path is not None
    assert not active_path.exists()
    assert final_path.name == "01_gripper_quicktest_20260611_183015_0.500s.csv"


def test_current_log_rotation_keeps_latest_five_final_files(tmp_path) -> None:
    writer = CurrentLogWriter(demo_name="02_pv_control", log_root=tmp_path, max_files=5)
    base = datetime(2026, 6, 11, 17, 0, 0)

    for index in range(6):
        writer.append_sample(10.0 + index * 10.0, 1.0 + index)
        writer.finalize(ended_at=base + timedelta(seconds=index))

    files = sorted((tmp_path / "02_pv_control").glob("*.csv"))
    names = {path.name for path in files}
    assert len(files) == 5
    assert all("_active_" not in path.name for path in files)
    assert "02_pv_control_20260611_170000_0.000s.csv" not in names
    assert "02_pv_control_20260611_170005_0.000s.csv" in names


def test_current_log_active_files_are_cleaned_on_startup(tmp_path) -> None:
    log_dir = tmp_path / "03_mit_linkage_force_control"
    log_dir.mkdir(parents=True)
    stale_active = log_dir / "03_mit_linkage_force_control_active_20260611_180000.csv"
    stale_active.write_text("partial\n", encoding="utf-8")
    final_log = log_dir / "03_mit_linkage_force_control_20260611_180001_1.000s.csv"
    final_log.write_text("complete\n", encoding="utf-8")

    CurrentLogWriter(demo_name="03_mit_linkage_force_control", log_root=tmp_path)

    assert not stale_active.exists()
    assert final_log.exists()

