import json

import pytest

from map_service import DoorStore


def test_add_persists_and_assigns_id(tmp_path):
    p = tmp_path / 'sala.doors.json'
    ds = DoorStore(str(p))
    d = ds.add([1.0, 2.0], [1.9, 2.0])
    assert d['id'] == 1
    on_disk = json.loads(p.read_text())
    assert on_disk['doors'][0]['a'] == [1.0, 2.0]


def test_add_validates_width(tmp_path):
    ds = DoorStore(str(tmp_path / 'x.doors.json'))
    with pytest.raises(ValueError):
        ds.add([0.0, 0.0], [0.1, 0.0])      # 0.1 m: estreito demais
    with pytest.raises(ValueError):
        ds.add([0.0, 0.0], [3.0, 0.0])      # 3 m: não é porta


def test_remove_and_reload(tmp_path):
    p = tmp_path / 'sala.doors.json'
    ds = DoorStore(str(p))
    d1 = ds.add([0.0, 0.0], [1.0, 0.0])
    ds.add([5.0, 0.0], [6.0, 0.0])
    assert ds.remove(d1['id']) is True
    assert ds.remove(99) is False
    ds2 = DoorStore(str(p))                  # recarrega do disco
    assert len(ds2.doors) == 1
    assert ds2.doors[0]['id'] == 2


def test_payload_shape(tmp_path):
    ds = DoorStore(str(tmp_path / 'x.doors.json'))
    ds.add([0.0, 0.0], [1.0, 0.0])
    pl = json.loads(ds.payload())
    assert pl == {'doors': [{'id': 1, 'a': [0.0, 0.0], 'b': [1.0, 0.0]}]}


def test_corrupt_file_starts_empty(tmp_path):
    p = tmp_path / 'bad.doors.json'
    p.write_text('{nope')
    ds = DoorStore(str(p))
    assert ds.doors == []
