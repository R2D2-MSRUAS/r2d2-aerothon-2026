"""Small stand-in for decoded pymavlink message objects used by unit tests."""


class FakeMessage:
    def __init__(self, message_type, source_system=1, source_component=1, **fields):
        self._message_type = message_type
        self._source_system = source_system
        self._source_component = source_component
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return self._source_system

    def get_srcComponent(self):
        return self._source_component
