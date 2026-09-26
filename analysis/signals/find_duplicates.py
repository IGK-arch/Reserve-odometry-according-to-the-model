"""Detect exact duplicated bags and input-only duplicates; read-only on dataset."""

import csv
import hashlib
import sqlite3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent

rows = []
for path in sorted((ROOT / 'dataset' / 'data').glob('*/*.db3')):
    file_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    conn = sqlite3.connect(f'file:{path.as_posix()}?mode=ro', uri=True)
    try:
        ids = {name: i for i, name in conn.execute('select id,name from topics')}
        input_names = ['/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity', '/vehicle/driver_position_cmd']
        selected = [ids[name] for name in input_names]
        content_hash = hashlib.sha256()
        for topic_id, stamp, data in conn.execute(
            f"select topic_id,timestamp,data from messages where topic_id in ({','.join(map(str,selected))}) order by timestamp"
        ):
            content_hash.update(input_names[selected.index(topic_id)].encode())
            content_hash.update(stamp.to_bytes(8, 'little', signed=True))
            content_hash.update(data)
        rows.append({'bag': path.parent.name, 'file_sha256': file_hash, 'inputs_sha256': content_hash.hexdigest()})
    finally:
        conn.close()

with (OUT / 'duplicates.csv').open('w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
