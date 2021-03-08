from typing import Dict


def update_message_from_dict(message: object, dic: Dict):
    for [k, v] in dic.items():
        if isinstance(v, dict):
            update_message_from_dict(message.__getattribute__(k), v)
        else:
            message.__setattr__(k, v)
