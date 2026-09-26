import glob
import os
import sqlite3
import struct


class CDR:
    def __init__(self, data):
        self.data = data
        self.off = 4
        self.endian = '<' if data[1] == 1 else '>'

    def value(self, code, alignment=None):
        size = struct.calcsize(code)
        alignment = alignment or size
        self.off += (-(self.off - 4)) % alignment
        result = struct.unpack_from(self.endian + code, self.data, self.off)[0]
        self.off += size
        return result

    def string(self):
        n = self.value('I')
        value = self.data[self.off:self.off+n]
        self.off += n
        return value[:-1].decode('utf-8')


def header(c):
    return c.value('i'), c.value('I'), c.string()


def fix(data):
    c = CDR(data)
    h = header(c)
    status = c.value('b')
    service = c.value('H')
    lat, lon, alt = (c.value('d') for _ in range(3))
    cov = tuple(c.value('d') for _ in range(9))
    cov_type = c.value('B')
    return h, status, service, lat, lon, alt, cov, cov_type, len(data), c.off


def vel(data):
    c = CDR(data)
    h = header(c)
    return h, tuple(c.value('d') for _ in range(6)), len(data), c.off


if __name__ == '__main__':
    for path in glob.glob('dataset/data/*/*.db3'):
        db = sqlite3.connect(path)
        topics = {name: ident for ident, name in db.execute('select id,name from topics')}
        q = db.execute('select timestamp,data from messages where topic_id=? limit 1', (topics['/sensing/gnss/master/fix'],)).fetchone()
        if q:
            print(os.path.basename(os.path.dirname(path)), q[0], fix(q[1]))
            q2 = db.execute('select timestamp,data from messages where topic_id=? limit 1', (topics['/sensing/gnss/master/vel'],)).fetchone()
            print(q2[0], vel(q2[1]))
            break
