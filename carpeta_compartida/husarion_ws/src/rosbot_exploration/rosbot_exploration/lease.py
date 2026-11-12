"""Persist ownership before sending a goal, including the acceptance race."""
import json
import os


def write_lease(path, action, uuid):
    if path:
        temporary = path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump({'action': action, 'uuid': bytes(uuid).hex()}, stream)
        os.replace(temporary, path)


def clear_lease(path, uuid):
    if not path:
        return
    try:
        with open(path, encoding='utf-8') as stream:
            lease = json.load(stream)
        if lease['uuid'] == bytes(uuid).hex():
            os.unlink(path)
    except FileNotFoundError:
        pass
