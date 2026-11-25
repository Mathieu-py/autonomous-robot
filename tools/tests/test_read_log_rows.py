import pytest

from chart_style import read_log_rows


def test_reads_csv_rows_from_a_putty_log(tmp_path):
    log = tmp_path / "putty.log"
    log.write_text("Safety test starts in 3 s\n"
                   "t_ms,speed,stop\n"
                   "20,0.500,none\n"
                   "Done\n"
                   "40,0.510,low battery\r\n")
    assert read_log_rows(log, 3, text_columns=1) == [
        [20.0, 0.5, "none"], [40.0, 0.51, "low battery"]]


def test_skips_rows_with_the_wrong_column_count_or_text(tmp_path):
    log = tmp_path / "putty.log"
    log.write_text("1,2,3\n1,2\n1,x,3\n1,2,3,4\n")
    assert read_log_rows(log, 3) == [[1.0, 2.0, 3.0]]


def test_exits_when_there_are_no_rows(tmp_path):
    log = tmp_path / "putty.log"
    log.write_text("Waiting for serial\n")
    with pytest.raises(SystemExit):
        read_log_rows(log, 3)
